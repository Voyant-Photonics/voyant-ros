
#include "voyant-ros/bin_to_mcap.hpp"

int main(int argc, char *argv[])
{
  if(argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <input_bin_file> <output_mcap_path>\n";
    return EXIT_FAILURE;
  }

  std::string input_file_path = argv[1];  // bin file
  std::string output_file_path = argv[2]; // mcap file. No extension needed

  // Create the playback instance
  VoyantPlayback player;
  if(!player.isValid())
  {
    std::cerr << "Failed to create VoyantPlayback instance: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  // Open the file
  std::cout << "Opening file: " << input_file_path << std::endl;
  if(!player.openFile(input_file_path))
  {
    std::cerr << "Failed to open file: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  // Initialize mcap writer
  auto writer = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_file_path; // No extension needed
  storage_options.storage_id = "mcap";    // Use MCAP format

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer->open(storage_options, converter_options);

  // Initialize LiDAR converter
  Bin2Mcap converter;

  // Read and process frames - timing and loopback is handled automatically
  while(player.nextFrame())
  {
    // Get metadata about the frame
    size_t frameIndex = player.currentFrameIndex();
    uint64_t timestamp = player.currentFrameTimestamp();

    // Access latest frame as a const reference
    const VoyantFrameWrapper &frame = player.currentFrame();
    sensor_msgs::msg::PointCloud2 cloud = converter.pointDatatoRosMsg(frame);
    std::cout << "ROS Cloud msg: " << cloud.height * cloud.width << std::endl;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&cloud, &serialized_msg);

    writer->write(serialized_msg, "/point_cloud", "sensor_msgs/msg/PointCloud2", cloud.header.stamp);

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

  return 0;
}