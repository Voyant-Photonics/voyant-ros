
// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/bin_to_mcap.hpp"

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <input_yaml_file_path>\n";
    return EXIT_FAILURE;
  }

  const std::string yaml_file_path = argv[1]; // yaml_file

  // Initialize LiDAR converter
  voyant_ros::Bin2Mcap converter(yaml_file_path);

  // Create the playback instance
  VoyantPlayback player;
  if(!player.isValid())
  {
    std::cerr << "Failed to create VoyantPlayback instance: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  // Open the file
  std::cout << "Opening file: " << converter.config.bin_input << std::endl;
  if(!player.openFile(converter.config.bin_input))
  {
    std::cerr << "Failed to open file: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  if(converter.config.mcap_output.empty())
  {
    std::cerr << "Output path cannot be empty" << std::endl;
    return EXIT_FAILURE;
  }

  // Initialize mcap writer
  auto writer = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = converter.config.mcap_output;       // No extension needed
  storage_options.storage_id = converter.config.storage_id; // Use MCAP format

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = converter.config.serialization_format;
  converter_options.output_serialization_format = converter.config.serialization_format;

  writer->open(storage_options, converter_options);

  rosbag2_storage::TopicMetadata topic_metadata;
  topic_metadata.name = converter.config.topic_name;
  topic_metadata.type = "sensor_msgs/msg/PointCloud2";
  topic_metadata.serialization_format = converter.config.serialization_format; // Must match your
                                                                               // converter_options

  writer->create_topic(topic_metadata);

  // Read and process frames - timing and loopback is handled automatically
  while(player.nextFrame())
  {
    // Get metadata about the frame
    size_t frameIndex = player.currentFrameIndex();
    uint64_t timestamp = player.currentFrameTimestamp();

    // Access latest frame as a const reference
    const VoyantFrameWrapper &frame = player.currentFrame();
    sensor_msgs::msg::PointCloud2 cloud = converter.pointDatatoRosMsg(frame);
    std::cout << "Pointcloud Size: " << cloud.height * cloud.width << std::endl;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&cloud, &serialized_msg);

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
#if defined(ROS2_ROLLING) || defined(ROS2_JAZZY)
    bag_msg->recv_timestamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
#else
    bag_msg->time_stamp = rclcpp::Time(cloud.header.stamp).nanoseconds();
#endif
    bag_msg->topic_name = "/point_cloud";
    bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();

    rcutils_allocator_t allocator = rcutils_get_default_allocator();

    auto ret = rcutils_uint8_array_init(bag_msg->serialized_data.get(),
                                        serialized_msg.get_rcl_serialized_message().buffer_capacity,
                                        &allocator);
    if(ret != RCUTILS_RET_OK)
    {

      std::cerr << "Error converting data" << std::endl;
      return EXIT_FAILURE;
    }
    memcpy(bag_msg->serialized_data->buffer,
           serialized_msg.get_rcl_serialized_message().buffer,
           serialized_msg.get_rcl_serialized_message().buffer_length);

    bag_msg->serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;

    writer->write(bag_msg);

    // Print frame metadata & frame debug string
    std::cout << "###############" << std::endl;
    std::cout << "Writing frame " << frameIndex << " (timestamp: " << std::fixed
              << std::setprecision(3) << timestamp / 1000000000.0 << "s)" << std::endl;
  }

  // Check if we exited the loop due to an error
  if(!player.getLastError().empty())
  {
    std::cerr << "Error during playback: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  // Print some details about the playback
  std::cout << "\nConversion complete!" << std::endl;
  std::cout << "Processed " << player.getFramesProcessed() << " frames" << std::endl;
  std::cout << "Saved output at: " << converter.config.mcap_output << std::endl;

  return 0;
}
