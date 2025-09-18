// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/msg/voyant_device_metadata.hpp"
#include "voyant_ros/point_types.hpp"
#include "voyant_ros/sensor_params.hpp"
#include <rclcpp/rclcpp.hpp>
#include <voyant_frame_wrapper.hpp>

namespace voyant_ros
{

/**
 * @brief Helper to fill point fields from frame data
 * Base template - specialized for each point type
 */
template <typename PointT>
inline void fillPointFromFrame(PointT &point, const PointDataWrapper &p, const VoyantFrameWrapper &frame)
{
  // This should never be called - specializations handle all cases
  static_assert(sizeof(PointT) == 0, "Unsupported point type - please provide a specialization");
}

/**
 * @brief Specialization for Standard MDL VoyantPoint
 */
template <>
inline void fillPointFromFrame<VoyantPoint>(VoyantPoint &point,
                                            const PointDataWrapper &p,
                                            const VoyantFrameWrapper & // unused
)
{
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  point.v = p.radial_vel();
  point.snr = p.snr_linear();
  point.drop_reason = static_cast<uint8_t>(p.drop_reason());
  point.timestamp_nsecs = p.timestamp_nanosecs();
  point.point_idx = p.point_index();
}

/**
 * @brief Specialization for VoyantPointMdlExtended
 */
template <>
inline void fillPointFromFrame<VoyantPointMdlExtended>(VoyantPointMdlExtended &point,
                                                       const PointDataWrapper &p,
                                                       const VoyantFrameWrapper &frame)
{
  // Fill all base fields
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  point.v = p.radial_vel();
  point.snr = p.snr_linear();
  point.drop_reason = static_cast<uint8_t>(p.drop_reason());
  point.timestamp_nsecs = p.timestamp_nanosecs();
  point.point_idx = p.point_index();
  point.calibrated_reflectance = p.calibrated_reflectance();
  point.noise_mean_estimate = p.noise_mean_estimate();
  point.min_ramp_snr = p.min_ramp_snr();
  point.frame_index = frame.header().frameIndex();
}

/**
 * @brief Generic frame to PointCloud2 converter
 */
template <typename PointT, typename FrameT, typename ConfigT>
sensor_msgs::msg::PointCloud2 convertFrameToPointCloud2(const FrameT &frame, const ConfigT &config)
{
  pcl::PointCloud<PointT> pcl_cloud;
  const auto &points = frame.points();
  size_t point_count = points.size();

  pcl_cloud.resize(point_count);
  pcl_cloud.width = point_count;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  size_t index = 0;
  for(const auto &p : points)
  {
    if(config.valid_only_filter && p.drop_reason() != DropReason::SUCCESS)
    {
      continue;
    }

    PointT &point = pcl_cloud.points[index++];
    fillPointFromFrame(point, p, frame); // Use fillPointFromFrame for PointT's type
  }
  pcl_cloud.resize(index);

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  switch(static_cast<TimestampMode>(config.timestamp_mode))
  {
    case TimestampMode::TIME_FROM_SENSOR:
      ros_cloud.header.stamp.sec = frame.header().timestampSeconds();
      ros_cloud.header.stamp.nanosec = frame.header().timestampNanoseconds();
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
inline sensor_msgs::msg::PointCloud2 convertFrameByFormat(const VoyantFrameWrapper &frame,
                                                          const SensorParams &config)
{
  switch(config.point_format)
  {
    case PointFormat::MDL_STANDARD:
      return convertFrameToPointCloud2<VoyantPoint>(frame, config);

    case PointFormat::MDL_EXTENDED:
      return convertFrameToPointCloud2<VoyantPointMdlExtended>(frame, config);

    case PointFormat::UNKNOWN:
      throw std::runtime_error("Point format not specified");

    default:
      throw std::runtime_error("Invalid point format");
  }
}

/**
 * @brief Convert PointCloud2 back to VoyantFrameWrapper
 * Template specialization for VoyantPointMdlExtended only
 * Expect other formats to be supported in the future
 */
template <typename PointT>
VoyantFrameWrapper convertPointCloud2ToFrame(const sensor_msgs::msg::PointCloud2 &cloud,
                                             const voyant_ros::msg::VoyantDeviceMetadata &metadata)
{
  static_assert(
      std::is_same_v<PointT, VoyantPointMdlExtended>,
      "Only VoyantPointMdlExtended is currently supported for PointCloud2 to frame conversion");

  // Convert ROS PointCloud2 to PCL
  pcl::PointCloud<PointT> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);

  VoyantFrameWrapper frame;

  // Setup header from metadata and cloud header
  VoyantHeaderWrapper &header = frame.headerMut();
  header.setMessageType(MessageType::VOYANT_FRAME);
  header.setTimestampSeconds(cloud.header.stamp.sec);
  header.setTimestampNanoseconds(cloud.header.stamp.nanosec);
  if(!pcl_cloud.empty())
  {
    // We will confirm that all points have the same frame index
    header.setFrameIndex(pcl_cloud.points[0].frame_index);
  }
  else
  {
    std::cerr << "Attempting conversion of empty PointCloud2 into VoyantFrame" << std::endl;
  }
  header.setDeviceClass(deviceClassFromDeviceId(metadata.device_id));
  header.setDeviceNumber(deviceNumberFromDeviceId(metadata.device_id));
  header.protoVersionMut() = VoyantVersionWrapper::fromU32Hash(metadata.proto_version_hash);
  header.apiVersionMut() = VoyantVersionWrapper::fromU32Hash(metadata.api_version_hash);
  header.firmwareVersionMut() = VoyantVersionWrapper::fromU32Hash(metadata.firmware_version_hash);
  header.hdlVersionMut() = VoyantVersionWrapper::fromU32Hash(metadata.hdl_version_hash);

  // Convert points
  std::vector<PointDataWrapper> &points = frame.pointsMut();
  points.reserve(pcl_cloud.size());

  for(const auto &pcl_point : pcl_cloud.points)
  {
    if(pcl_point.frame_index != header.frameIndex())
    {
      std::cerr << "Warning: Skipping point with inconsistent frame_index. Expected: "
                << header.frameIndex() << ", Found: " << pcl_point.frame_index << std::endl;
      continue;
    }

    // Fill point data from PCL point
    PointDataWrapper point_wrapper;
    point_wrapper.set_x(pcl_point.x);
    point_wrapper.set_y(pcl_point.y);
    point_wrapper.set_z(pcl_point.z);
    point_wrapper.set_radial_vel(pcl_point.v);
    point_wrapper.set_snr_linear(pcl_point.snr);
    point_wrapper.set_drop_reason(static_cast<DropReason>(pcl_point.drop_reason));
    point_wrapper.set_timestamp_nanosecs(pcl_point.timestamp_nsecs);
    point_wrapper.set_point_index(pcl_point.point_idx);
    point_wrapper.set_calibrated_reflectance(pcl_point.calibrated_reflectance);
    point_wrapper.set_noise_mean_estimate(pcl_point.noise_mean_estimate);
    point_wrapper.set_min_ramp_snr(pcl_point.min_ramp_snr);

    points.push_back(std::move(point_wrapper));
  }

  return frame;
}

/**
 * @brief Convenience function with explicit template instantiation
 */
inline VoyantFrameWrapper convertMdlExtendedPointCloud2ToFrame(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const voyant_ros::msg::VoyantDeviceMetadata &metadata)
{
  return convertPointCloud2ToFrame<VoyantPointMdlExtended>(cloud, metadata);
}

} // namespace voyant_ros
