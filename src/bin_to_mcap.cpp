// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/bin_to_mcap.hpp"

namespace voyant_ros
{

Bin2Mcap::Bin2Mcap(const std::string &yaml_path)
{
  // Load the params from the yaml file
  config = Bin2Mcap::load_conversion_params(yaml_path);
}

Bin2Mcap::~Bin2Mcap() { std::cout << "Shutting down..." << std::endl; };

SensorParams Bin2Mcap::load_conversion_params(const std::string &yaml_path)
{
  SensorParams params;

  try
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto sensor_config = config["/**"]["ros__parameters"];

    if(!sensor_config)
    {
      throw std::runtime_error("Missing 'sensor_params' section");
    }
    params.bin_input = sensor_config["bin_input"].as<std::string>();
    params.mcap_output = sensor_config["mcap_output"].as<std::string>();
    params.lidar_frame_id = sensor_config["frame_id"].as<std::string>();
    params.timestamp_mode = sensor_config["timestamp_mode"].as<int>();
    params.valid_only_filter = sensor_config["valid_only_filter"].as<bool>();
    params.storage_id = sensor_config["storage_id"].as<std::string>();
    params.serialization_format = sensor_config["serialization_format"].as<std::string>();
    params.topic_name = sensor_config["topic_name"].as<std::string>();
  }
  catch(const std::exception &e)
  {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return params;
}

sensor_msgs::msg::PointCloud2 Bin2Mcap::pointDatatoRosMsg(const VoyantFrameWrapper frame)
{
  return convertFrameToPointCloud2<VoyantPoint>(frame, this->config);
}
} // namespace voyant_ros
