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
  TIME_FROM_SENSOR = 0,
  TIME_FROM_ROS = 1,
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
};

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
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.v = p.radial_vel();
    point.snr = p.snr_linear();
    point.drop_reason = static_cast<uint8_t>(p.drop_reason());
    point.timestamp_nsecs = p.timestamp_nanosecs();
    point.point_idx = p.point_index();
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

} // namespace voyant_ros
