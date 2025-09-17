// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/mcap_to_bin.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace voyant_ros
{

Mcap2Bin::Mcap2Bin(const std::string &yaml_path) { config = load_conversion_params(yaml_path); }

Mcap2Bin::~Mcap2Bin() { std::cout << "Shutting down..." << std::endl; }

McapConfig Mcap2Bin::load_conversion_params(const std::string &yaml_path)
{
  McapConfig params;

  try
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    params.mcap_input = config["mcap_input"].as<std::string>();
    params.bin_output = config["bin_output"].as<std::string>();
  }
  catch(const std::exception &e)
  {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return params;
}

bool Mcap2Bin::validatePointCloudFormat(const sensor_msgs::msg::PointCloud2 &cloud)
{
  // TODO: Fix this test to be fully robust

  // Check for VoyantPointMdlExtended format - look for the extended fields
  bool has_calibrated_reflectance = false;
  bool has_frame_index = false;

  for(const auto &field : cloud.fields)
  {
    if(field.name == "calibrated_reflectance")
    {
      has_calibrated_reflectance = true;
    }
    else if(field.name == "frame_index")
    {
      has_frame_index = true;
    }
  }

  return has_calibrated_reflectance && has_frame_index;
}

} // namespace voyant_ros
