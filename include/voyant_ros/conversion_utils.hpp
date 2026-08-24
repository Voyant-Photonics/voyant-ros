// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/msg/voyant_device_metadata.hpp"
#include "voyant_ros/point_types.hpp"
#include "voyant_ros/sensor_params.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <voyant_frame.hpp>

namespace voyant_ros
{

/**
 * @brief Helper to fill point fields from frame data
 * Base template - specialized for each point type
 */
template <typename PointT>
inline void fillPointFromFrame(PointT &point,
                               const PointData &p,
                               const VoyantVec3 &xyz,
                               const VoyantFrame &frame)
{
  // This should never be called - specializations handle all cases
  static_assert(sizeof(PointT) == 0, "Unsupported point type - please provide a specialization");
}

/**
 * @brief Specialization for the standard VoyantPoint
 */
template <>
inline void fillPointFromFrame<VoyantPoint>(VoyantPoint &point,
                                            const PointData &p,
                                            const VoyantVec3 &xyz,
                                            const VoyantFrame & // unused
)
{
  point.x = xyz.x;
  point.y = xyz.y;
  point.z = xyz.z;
  point.v = p.doppler_mps;
  point.snr = p.snr;
  point.drop_reason = p.drop_reason;
  point.timestamp_nsecs = p.timestamp_nanosecs;
  point.point_idx = (static_cast<uint32_t>(p.azimuth_idx) << 16) | p.elevation_idx;
}

/**
 * @brief Specialization for VoyantPointExtended
 */
template <>
inline void fillPointFromFrame<VoyantPointExtended>(VoyantPointExtended &point,
                                                    const PointData &p,
                                                    const VoyantVec3 &xyz,
                                                    const VoyantFrame &frame)
{
  // Fill all base fields
  point.x = xyz.x;
  point.y = xyz.y;
  point.z = xyz.z;
  point.v = p.doppler_mps;
  point.snr = p.snr;
  point.drop_reason = p.drop_reason;
  point.timestamp_nsecs = p.timestamp_nanosecs;
  point.point_idx = (static_cast<uint32_t>(p.azimuth_idx) << 16) | p.elevation_idx;
  point.calibrated_reflectance = p.calibrated_reflectance;
  point.frame_index = frame.frameIndex();
}

/**
 * @brief Generic frame to PointCloud2 converter
 *
 * Frames contain invalid returns only when the source keeps them (client
 * diagnostic mode / playback keep_invalid_points), so every point is published.
 * Per-point timestamps are offsets from the frame stamp in the cloud header.
 */
template <typename PointT, typename ConfigT>
sensor_msgs::msg::PointCloud2 convertFrameToPointCloud2(const VoyantFrame &frame, const ConfigT &config)
{
  const std::vector<PointData> &points = frame.points();
  const std::vector<VoyantVec3> xyz = frame.xyz(); // index-aligned with points()

  pcl::PointCloud<PointT> pcl_cloud;
  pcl_cloud.resize(points.size());
  pcl_cloud.width = points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  for(size_t i = 0; i < points.size(); i++)
  {
    fillPointFromFrame(pcl_cloud.points[i], points[i], xyz[i], frame);
  }

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  switch(static_cast<TimestampMode>(config.timestamp_mode))
  {
    case TimestampMode::TIME_FROM_SENSOR:
      ros_cloud.header.stamp.sec = static_cast<int32_t>(frame.timestampSeconds());
      ros_cloud.header.stamp.nanosec = static_cast<uint32_t>(frame.timestampNanoseconds());
      break;

    case TimestampMode::TIME_FROM_ROS:
      ros_cloud.header.stamp = rclcpp::Clock().now();
      break;

    default:
      throw std::runtime_error(
          "Unknown timestamp_mode enum value: " + std::to_string(config.timestamp_mode));
  }

  ros_cloud.header.frame_id = config.lidar_frame_id;
  return ros_cloud;
}

/**
 * @brief Factory function to convert frame based on configured point format
 * This allows runtime selection of point format
 */
inline sensor_msgs::msg::PointCloud2 convertFrameByFormat(const VoyantFrame &frame,
                                                          const SensorParams &config)
{
  switch(config.point_format)
  {
    case PointFormat::STANDARD:
      return convertFrameToPointCloud2<VoyantPoint>(frame, config);

    case PointFormat::EXTENDED:
      return convertFrameToPointCloud2<VoyantPointExtended>(frame, config);

    case PointFormat::UNKNOWN:
      throw std::runtime_error("Point format not specified");

    default:
      throw std::runtime_error("Invalid point format");
  }
}

/**
 * @brief Convert PointCloud2 back to a recordable VoyantFrame
 * Template specialization for VoyantPointExtended only
 * Expect other formats to be supported in the future
 *
 * The rebuilt frame is a synthetic frame: it keeps the cloud's timeline
 * (timestamps and frame index) but cannot claim the original device identity,
 * and carries no sensor state. combine_method and user_data are not carried
 * through the point cloud, so they are zeroed.
 */
template <typename PointT>
VoyantFrame convertPointCloud2ToFrame(const sensor_msgs::msg::PointCloud2 &cloud,
                                      const voyant_ros::msg::VoyantDeviceMetadata &metadata)
{
  static_assert(
      std::is_same_v<PointT, VoyantPointExtended>,
      "Only VoyantPointExtended is currently supported for PointCloud2 to frame conversion");

  // Convert ROS PointCloud2 to PCL
  pcl::PointCloud<PointT> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);

  uint32_t frame_index = 0;
  if(!pcl_cloud.empty())
  {
    // We will confirm that all points have the same frame index
    frame_index = pcl_cloud.points[0].frame_index;
  }
  else
  {
    std::cerr << "Attempting conversion of empty PointCloud2 into VoyantFrame" << std::endl;
  }

  std::vector<PointData> points;
  points.reserve(pcl_cloud.size());

  for(const auto &pcl_point : pcl_cloud.points)
  {
    if(pcl_point.frame_index != frame_index)
    {
      std::cerr << "Warning: Skipping point with inconsistent frame_index. Expected: " << frame_index
                << ", Found: " << pcl_point.frame_index << std::endl;
      continue;
    }

    PointData p{};
    // Invert the sensor-frame projection (+x forward, +y left, +z up) back to the
    // stored spherical fields; a zero-length position stays at range 0.
    const float range =
        std::sqrt(pcl_point.x * pcl_point.x + pcl_point.y * pcl_point.y + pcl_point.z * pcl_point.z);
    if(range > std::numeric_limits<float>::epsilon())
    {
      p.range_m = range;
      p.azimuth_rad = std::atan2(pcl_point.y, pcl_point.x);
      p.elevation_rad = std::asin(pcl_point.z / range);
    }
    p.doppler_mps = pcl_point.v;
    p.snr = pcl_point.snr;
    p.calibrated_reflectance = pcl_point.calibrated_reflectance;
    p.timestamp_nanosecs = pcl_point.timestamp_nsecs;
    p.azimuth_idx = static_cast<uint16_t>(pcl_point.point_idx >> 16);
    p.elevation_idx = static_cast<uint16_t>(pcl_point.point_idx & 0xFFFF);
    p.drop_reason = pcl_point.drop_reason;

    points.push_back(p);
  }

  VoyantFrame::SyntheticFrameDesc desc;
  desc.timestampSeconds = cloud.header.stamp.sec;
  desc.timestampNanoseconds = static_cast<int32_t>(cloud.header.stamp.nanosec);
  desc.frameIndex = frame_index;

  std::optional<VoyantFrame> frame = VoyantFrame::synthetic(std::move(points), desc);
  if(!frame)
  {
    throw std::runtime_error(
        "Failed to rebuild a frame from PointCloud2 (device: " + metadata.device_id + ")");
  }
  return std::move(*frame);
}

/**
 * @brief Convenience function with explicit template instantiation
 */
inline VoyantFrame convertExtendedPointCloud2ToFrame(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const voyant_ros::msg::VoyantDeviceMetadata &metadata)
{
  return convertPointCloud2ToFrame<VoyantPointExtended>(cloud, metadata);
}

} // namespace voyant_ros
