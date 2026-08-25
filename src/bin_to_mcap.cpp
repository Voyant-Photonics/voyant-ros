// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/bin_to_mcap.hpp"
#include <yaml-cpp/yaml.h>

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

    // yaml-cpp reports a missing key as a bare "bad conversion", so name it instead.
    for(const char *key : {"bin_input",
                           "mcap_output",
                           "topic_name",
                           "frame_id",
                           "storage_id",
                           "serialization_format",
                           "timestamp_mode",
                           "diagnostic_mode"})
    {
      if(!config[key])
      {
        throw std::runtime_error(std::string("missing key '") + key + "'");
      }
    }

    params.bin_input = config["bin_input"].as<std::string>();
    params.mcap_output = config["mcap_output"].as<std::string>();
    params.lidar_frame_id = config["frame_id"].as<std::string>();
    params.timestamp_mode = config["timestamp_mode"].as<int>();
    params.diagnostic_mode = config["diagnostic_mode"].as<bool>();
    params.storage_id = config["storage_id"].as<std::string>();
    params.serialization_format = config["serialization_format"].as<std::string>();
    params.topic_name = config["topic_name"].as<std::string>();
  }
  catch(const std::exception &e)
  {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return params;
}

sensor_msgs::msg::PointCloud2 Bin2Mcap::pointDatatoRosMsg(const VoyantFrame &frame)
{
  return convertFrameToPointCloud2(frame, config);
}
} // namespace voyant_ros
