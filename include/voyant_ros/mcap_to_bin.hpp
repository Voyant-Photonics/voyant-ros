// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/msg/voyant_device_metadata.hpp"
#include <memory>
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
 * @brief MCAP playback manager for two-phase processing
 */
class McapPlayback
{
public:
  McapPlayback(const McapConfig &config);
  ~McapPlayback() = default;

  /**
   * @brief First pass - validate metadata and first frame format
   * @return true if validation passed, false otherwise
   */
  bool validate();

  /**
   * @brief Second pass - restart from beginning and process all frames
   * @return true if processing / conversion passed, false otherwise
   */
  bool processFrames();

  /**
   * @brief Get the found metadata
   */
  const voyant_ros::msg::VoyantDeviceMetadata &getMetadata() const { return metadata_; }

private:
  McapConfig config_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_cpp::ConverterOptions converter_options_;
  voyant_ros::msg::VoyantDeviceMetadata metadata_;
  bool validated_;

  void openReader();
  bool is_voyant_extended_format(const sensor_msgs::msg::PointCloud2 &cloud);
};

/**
 * @brief Load conversion parameters from YAML file
 */
McapConfig load_config(const std::string &yaml_path);

} // namespace voyant_ros
