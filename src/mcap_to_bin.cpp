// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/mcap_to_bin.hpp"
#include "voyant_ros/conversion_utils.hpp"
#include <iomanip>
#include <iostream>
#include <map>
#include <voyant_data_recorder.hpp>
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
    : config_(config),
      validated_(false)
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
    if(bag_message->topic_name.find(kDeviceMetadataTopic) != std::string::npos && !metadata_found)
    {
      try
      {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        metadata_serializer.deserialize_message(&serialized_msg, &metadata_);
        metadata_found = true;

        // api_version arrived with the v1.0.0 message; without it the bag predates this
        // driver, and its clouds carry a point layout this build cannot rebuild.
        if(metadata_.api_version.empty())
        {
          std::cerr << "✗ Metadata has no api_version: this MCAP was recorded by a "
                       "pre-1.0.0 driver and cannot be converted"
                    << std::endl;
          return false;
        }

        std::cout << "✓ Found metadata - Device ID: " << metadata_.device_id << " (recorded with "
                  << "voyant-api " << metadata_.api_version << ")" << std::endl;
      }
      catch(const std::exception &e)
      {
        // A pre-1.0.0 bag carries four version *hashes* where this build expects
        // strings, so it fails here rather than at the api_version check below.
        std::cerr << "✗ Failed to parse metadata: " << e.what()
                  << "\n  The MCAP was most likely recorded by a pre-1.0.0 driver, whose "
                     "VoyantDeviceMetadata and point layout this build cannot read."
                  << std::endl;
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

        if(contains_valid_format(cloud))
        {
          first_frame_validated = true;
          std::cout << "✓ First frame contains a valid point format (" << cloud.width * cloud.height
                    << " points)" << std::endl;
        }
        else
        {
          std::cerr << "✗ First frame does not contain a valid point format" << std::endl;
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
  validated_ = true;

  // Close reader after validation
  reader_.reset();

  return true;
}

bool McapPlayback::processFrames()
{
  if(!validated_)
  {
    std::cerr << "Do not process frames without first validating" << std::endl;
    return false;
  }

  // The recorder reports only valid/invalid, so check the extension here to have a
  // reason to give.
  if(config_.bin_output.size() < 5 ||
     config_.bin_output.compare(config_.bin_output.size() - 5, 5, ".vynt") != 0)
  {
    std::cerr << "Output path must end in .vynt: " << config_.bin_output << std::endl;
    return false;
  }

  // Create recorder with (mostly) default configuration
  VoyantRecorderConfig recorder_config(config_.bin_output);
  recorder_config.timestampFilename = false; // Turn time-stamping the filename off
  VoyantRecorder recorder(recorder_config);

  if(!recorder.isValid())
  {
    std::cerr << "Failed to create VoyantRecorder" << std::endl;
    return false;
  }

  // Restart from beginning
  openReader();

  std::cout << "\nProcessing frames..." << std::endl;

  size_t frame_count = 0;
  size_t recorded_count = 0;
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

        // Convert PointCloud2 back to VoyantFrame and record
        try
        {
          VoyantFrame frame = convertPointCloud2ToFrame(cloud, metadata_);

          RecordResult result = recorder.recordFrame(frame);
          if(result == RecordResult::Error || result == RecordResult::Unknown)
          {
            std::cerr << "\nError recording frame " << frame_count << std::endl;
            return false;
          }

          recorded_count++;
        }
        catch(const std::exception &e)
        {
          std::cerr << "\nFailed to convert frame " << frame_count
                    << " to VoyantFrame: " << e.what() << std::endl;
          return false;
        }

        frame_count++;
      }
      catch(const std::exception &e)
      {
        std::cerr << "Failed to parse frame " << frame_count << ": " << e.what() << std::endl;
        return false;
      }
    }
  }

  // Finalize recording
  std::cout << "\nFinalizing recording..." << std::endl;
  if(!recorder.finalize())
  {
    std::cerr << "Failed to finalize VoyantRecorder" << std::endl;
    return false;
  }

  std::cout << "\nProcessed " << frame_count << " frames" << std::endl;
  std::cout << "Successfully recorded " << recorded_count << " frames" << std::endl;
  std::cout << "Output saved to: " << config_.bin_output << std::endl;

  return true;
}

bool McapPlayback::contains_valid_format(const sensor_msgs::msg::PointCloud2 &cloud)
{
  // Types are checked because PCL maps a field only when name and datatype both match,
  // and silently leaves the destination zeroed otherwise.
  using Field = sensor_msgs::msg::PointField;
  const std::map<std::string, uint8_t> required_fields = {{"x", Field::FLOAT32},
                                                          {"y", Field::FLOAT32},
                                                          {"z", Field::FLOAT32},
                                                          {"v", Field::FLOAT32},
                                                          {"snr", Field::FLOAT32},
                                                          {"drop_reason", Field::UINT8},
                                                          {"timestamp_nsecs", Field::UINT32},
                                                          {"point_idx", Field::UINT32},
                                                          {"calibrated_reflectance", Field::FLOAT32},
                                                          {"frame_index", Field::UINT32}};

  std::map<std::string, uint8_t> found_fields;
  for(const auto &field : cloud.fields)
  {
    found_fields.emplace(field.name, field.datatype);
  }

  bool valid = true;
  for(const auto &[name, datatype] : required_fields)
  {
    auto found = found_fields.find(name);
    if(found == found_fields.end())
    {
      std::cout << "PointCloud missing required field: " << name << std::endl;
      valid = false;
    }
    else if(found->second != datatype)
    {
      std::cout << "PointCloud field '" << name << "' has type " << static_cast<int>(found->second)
                << ", expected " << static_cast<int>(datatype) << std::endl;
      valid = false;
    }
  }

  return valid;
}

} // namespace voyant_ros
