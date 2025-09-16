// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/point_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <voyant_client.hpp>
#include <voyant_frame_wrapper.hpp>

namespace voyant_ros
{

enum class TimestampMode
{
  // TODO: Breaking change - Add TimestampMode::UNKNOWN = 0
  TIME_FROM_SENSOR = 0,
  TIME_FROM_ROS = 1,
};

enum class PointFormat
{
  UNKNOWN = 0,
  MDL_STANDARD = 1,
  MDL_EXTENDED = 2,
};

/**
 * @brief Sensor configuration parameters
 */
struct SensorParams
{
  std::string mcap_output;
  std::string bin_input;
  std::string lidar_frame_id;
  int timestamp_mode;
  bool valid_only_filter;
  std::string storage_id;
  std::string serialization_format;
  std::string topic_name;
  std::string binding_address;
  std::string multicast_group;
  std::string interface_address;
  PointFormat point_format = PointFormat::UNKNOWN;
};

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

} // namespace voyant_ros
