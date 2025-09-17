// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/mcap_to_bin.hpp"
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace voyant_ros
{

McapConfig load_config(const std::string &yaml_path)
{
  McapConfig config;

  try
  {
    YAML::Node yaml_config = YAML::LoadFile(yaml_path);
    config.mcap_input = yaml_config["mcap_input"].as<std::string>();
    config.bin_output = yaml_config["bin_output"].as<std::string>();
  }
  catch(const std::exception &e)
  {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return config;
}

McapPlayback::McapPlayback(const McapConfig &config)
    : config_(config)
{
  storage_options_.uri = config_.mcap_input;
  storage_options_.storage_id = "mcap";

  converter_options_.input_serialization_format = "cdr";
  converter_options_.output_serialization_format = "cdr";
}

void McapPlayback::openReader()
{
  reader_ = std::make_unique<rosbag2_cpp::Reader>();

  try
  {
    reader_->open(storage_options_, converter_options_);
  }
  catch(const std::exception &e)
  {
    std::cerr << "Failed to open MCAP file: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

bool McapPlayback::validate()
{
  openReader();

  std::cout << "Validating file: " << config_.mcap_input << std::endl;

  bool metadata_found = false;
  bool first_frame_validated = false;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serializer;
  rclcpp::Serialization<voyant_ros::msg::VoyantDeviceMetadata> metadata_serializer;

  // First pass - look for metadata and validate first frame
  while(reader_->has_next() && (!metadata_found || !first_frame_validated))
  {
    auto bag_message = reader_->read_next();

    // Check for metadata message
    if(bag_message->topic_name.find("device_metadata") != std::string::npos && !metadata_found)
    {
      try
      {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        metadata_serializer.deserialize_message(&serialized_msg, &metadata_);
        metadata_found = true;

        std::cout << "✓ Found metadata - Device ID: " << metadata_.device_id << std::endl;
      }
      catch(const std::exception &e)
      {
        std::cerr << "✗ Failed to parse metadata: " << e.what() << std::endl;
        return false;
      }
    }

    // Check first point cloud frame
    if(bag_message->topic_name.find("point_cloud") != std::string::npos && !first_frame_validated)
    {
      try
      {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pc_serializer.deserialize_message(&serialized_msg, &cloud);

        if(is_voyant_extended_format(cloud))
        {
          first_frame_validated = true;
          std::cout << "✓ First frame is VoyantPointMdlExtended format ("
                    << cloud.width * cloud.height << " points)" << std::endl;
        }
        else
        {
          std::cerr << "✗ First frame is not VoyantPointMdlExtended format" << std::endl;
          return false;
        }
      }
      catch(const std::exception &e)
      {
        std::cerr << "✗ Failed to parse first frame: " << e.what() << std::endl;
        return false;
      }
    }
  }

  if(!metadata_found)
  {
    std::cerr << "✗ No metadata message found" << std::endl;
    return false;
  }

  if(!first_frame_validated)
  {
    std::cerr << "✗ No valid point cloud frames found" << std::endl;
    return false;
  }

  std::cout << "✓ Validation complete" << std::endl;

  // Close reader after validation
  reader_.reset();

  return true;
}

void McapPlayback::processFrames()
{
  // Restart from beginning
  openReader();

  std::cout << "\nProcessing frames..." << std::endl;

  size_t frame_count = 0;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serializer;

  // Second pass - process only point cloud frames
  while(reader_->has_next())
  {
    auto bag_message = reader_->read_next();

    // Process only point cloud messages
    if(bag_message->topic_name.find("point_cloud") != std::string::npos)
    {
      try
      {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pc_serializer.deserialize_message(&serialized_msg, &cloud);

        // Print frame metadata
        std::cout << "###############" << std::endl;
        std::cout << "Frame " << frame_count << " | timestamp: " << std::fixed << std::setprecision(3)
                  << cloud.header.stamp.sec + cloud.header.stamp.nanosec / 1e9 << "s"
                  << " | points: " << cloud.width * cloud.height << std::endl;

        // TODO: Add conversion logic here

        frame_count++;
      }
      catch(const std::exception &e)
      {
        std::cerr << "Failed to parse frame " << frame_count << ": " << e.what() << std::endl;
      }
    }
  }

  std::cout << "\nProcessed " << frame_count << " frames" << std::endl;
  std::cout << "Output will be saved at: " << config_.bin_output << std::endl;
}

bool McapPlayback::is_voyant_extended_format(const sensor_msgs::msg::PointCloud2 &cloud)
{
  // Check for VoyantPointMdlExtended format - look for the key extended fields
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
