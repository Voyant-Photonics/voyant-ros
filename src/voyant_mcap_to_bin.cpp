// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/mcap_to_bin.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <input_yaml_file_path>\n";
    return EXIT_FAILURE;
  }

  const std::string yaml_file_path = argv[1];

  // Initialize MCAP converter
  voyant_ros::Mcap2Bin converter(yaml_file_path);

  // Create the reader instance
  auto reader = std::make_unique<rosbag2_cpp::Reader>();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = converter.config.mcap_input;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try
  {
    reader->open(storage_options, converter_options);
  }
  catch(const std::exception &e)
  {
    std::cerr << "Failed to open MCAP file: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Opening file: " << converter.config.mcap_input << std::endl;

  if(converter.config.bin_output.empty())
  {
    std::cerr << "Output path cannot be empty" << std::endl;
    return EXIT_FAILURE;
  }

  // Look for metadata message first
  voyant_ros::msg::VoyantDeviceMetadata metadata;
  bool metadata_found = false;
  size_t frame_count = 0;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serializer;
  rclcpp::Serialization<voyant_ros::msg::VoyantDeviceMetadata> metadata_serializer;

  // Read and process messages
  while(reader->has_next())
  {
    auto bag_message = reader->read_next();

    // Check for metadata message
    if(bag_message->topic_name.find("metadata") != std::string::npos && !metadata_found)
    {
      try
      {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        metadata_serializer.deserialize_message(&serialized_msg, &metadata);
        metadata_found = true;

        std::cout << "=== Found Voyant Device Metadata ===" << std::endl;
        std::cout << "Device ID: " << metadata.device_id << std::endl;
        std::cout << "Proto Version Hash: 0x" << std::hex << metadata.proto_version_hash << std::endl;
        std::cout << "API Version Hash: 0x" << std::hex << metadata.api_version_hash << std::endl;
        std::cout << "Firmware Version Hash: 0x" << std::hex << metadata.firmware_version_hash
                  << std::endl;
        std::cout << "HDL Version Hash: 0x" << std::hex << metadata.hdl_version_hash << std::endl;
        std::cout << std::dec << "====================================" << std::endl;
      }
      catch(const std::exception &e)
      {
        std::cerr << "Failed to parse metadata: " << e.what() << std::endl;
      }
      continue;
    }

    // Check for point cloud messages
    if(bag_message->topic_name.find("point_cloud") != std::string::npos)
    {
      try
      {
        sensor_msgs::msg::PointCloud2 cloud;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        pc_serializer.deserialize_message(&serialized_msg, &cloud);

        // Validate that this contains VoyantPointMdlExtended points
        if(!converter.validatePointCloudFormat(cloud))
        {
          std::cout << "Warning: Skipping point cloud - not VoyantPointMdlExtended format" << std::endl;
          continue;
        }

        // Print frame header
        std::cout << "###############" << std::endl;
        std::cout << "Processing frame " << frame_count << " (timestamp: " << std::fixed
                  << std::setprecision(3)
                  << cloud.header.stamp.sec + cloud.header.stamp.nanosec / 1e9 << "s)" << std::endl;
        std::cout << "Points: " << cloud.width * cloud.height << std::endl;

        // TODO: Process the point cloud data here

        frame_count++;
      }
      catch(const std::exception &e)
      {
        std::cerr << "Failed to parse point cloud: " << e.what() << std::endl;
      }
    }
  }

  if(!metadata_found)
  {
    std::cout << "Warning: No metadata message found in MCAP file" << std::endl;
  }

  std::cout << "\nProcessing complete!" << std::endl;
  std::cout << "Processed " << frame_count << " frames" << std::endl;
  std::cout << "Output will be saved at: " << converter.config.bin_output << std::endl;

  return 0;
}
