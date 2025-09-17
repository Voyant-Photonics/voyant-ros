// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/msg/voyant_device_metadata.hpp"
#include <iomanip>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace voyant_ros
{

/**
 * @brief Simple configuration for MCAP to bin conversion
 */
struct McapConfig
{
  std::string mcap_input;
  std::string bin_output;
};

/**
 * @class Mcap2Bin
 * @brief Offline .mcap to .bin converter
 */
class Mcap2Bin
{
public:
  /**
   * @brief Construct a new Mcap to bin object
   */
  Mcap2Bin(const std::string &yaml_path);

  /**
   * @brief Destroy the Mcap to bin object
   */
  ~Mcap2Bin();

  /**
   * @brief Validate that point cloud contains VoyantPointMdlExtended format
   */
  bool validatePointCloudFormat(const sensor_msgs::msg::PointCloud2 &cloud);

  /**
   * @brief Load conversion parameters from YAML file
   */
  McapConfig load_conversion_params(const std::string &yaml_path);

  McapConfig config;
};

} // namespace voyant_ros
