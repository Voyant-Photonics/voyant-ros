// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/sensor_driver.hpp"
#include <iomanip>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <string>
#include <voyant_frame_wrapper.hpp>
#include <voyant_playback.hpp>
#include <yaml-cpp/yaml.h>

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
  Bin2Mcap(const std::string &yaml_path);

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

  SensorParams load_conversion_params(const std::string &yaml_path);
  SensorParams config;
};
