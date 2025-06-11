// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <voyant_frame_wrapper.hpp>
#include <voyant_playback.hpp>

/**
 * @brief Point structure for the Voyant LiDAR sensor
 *
 */
struct EIGEN_ALIGN16 VoyantPoint
{
  PCL_ADD_POINT4D; // This adds x, y, z, and padding
  float v;
  float snr;
  uint8_t drop_reason;
  int32_t timestamp_nsecs;
  uint32_t point_idx;

  inline VoyantPoint()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f; // Set padding to 1 to prevent undefined behavior.
    v = 0.0f;
    snr = 0.0f;
    drop_reason = 0;
    timestamp_nsecs = 0;
    point_idx = 0;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VoyantPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, v, v)(float, snr, snr)(uint8_t, drop_reason, drop_reason))

/**
 * @class Bin2Mcap
 * @brief Offline .bin to .mcap converter
 */
class Bin2Mcap
{
public:
  /**
   * @brief Construct a new Bin to mcap object
   */
  Bin2Mcap();

  /**
   * @brief Destroy the Bin to mcap object
   */
  ~Bin2Mcap();

  /**
   * @brief Convert the Voyant frame to a ROS2 message
   *
   * @param frame The Voyant frame
   * @return sensor_msgs::msg::PointCloud2 The ROS2 point cloud message
   */
  sensor_msgs::msg::PointCloud2 pointDatatoRosMsg(const VoyantFrameWrapper frame);

  rclcpp::Time chrono_to_ros_time(const std::chrono::system_clock::time_point &chrono_time);

private:
  // Configuration parameters
  std::string timestamp_mode_;
  std::string lidar_frame_id_;
  bool valid_only_filter_;
};
