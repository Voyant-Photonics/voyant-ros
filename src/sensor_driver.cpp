// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/sensor_driver.hpp"
#include <chrono>
#include <thread>

namespace voyant_ros
{

VoyantSensorDriver::VoyantSensorDriver()
    : Node("voyant_sensor_node")
{
  // ToDo: Write a sensor defined QoS profile
  getParams();
  initialize();
  // Note: Changing the default point cloud topic name will break the `voyant_foxglove_cfg.json`
  // file, but you can always remap the topic name or visualize the point cloud using different
  // config file in Foxglove or RViz.
  points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 100);

  // Latched publisher for static metadata
  auto metadata_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                          .durability(rclcpp::DurabilityPolicy::TransientLocal)
                          .reliability(rclcpp::ReliabilityPolicy::Reliable);

  metadata_pub =
      this->create_publisher<voyant_ros::msg::VoyantDeviceMetadata>("device_metadata", metadata_qos);

  publishPointCloud();
}

VoyantSensorDriver::~VoyantSensorDriver()
{
  // Stop the ros node
  RCLCPP_INFO(get_logger(), "[+] Shutting down the node");
  rclcpp::shutdown();
}

void VoyantSensorDriver::getParams()
{
  this->declare_parameter<std::string>("binding_address", "0.0.0.0:4444");
  this->declare_parameter<std::string>("multicast_group", "224.0.0.0");
  this->declare_parameter<std::string>("interface_address", "127.0.0.1");
  this->declare_parameter<bool>("spn_filter", true);
  this->declare_parameter<int>("timestamp_mode", 0); // Default to TIME_FROM_SENSOR (0)
  this->declare_parameter<std::string>("frame_id", "lidar_sensor");
  this->declare_parameter<int>("point_format", 1); // Default to MDL_STANDARD (1)

  config_.binding_address = this->get_parameter("binding_address").as_string();
  config_.multicast_group = this->get_parameter("multicast_group").as_string();
  config_.interface_address = this->get_parameter("interface_address").as_string();
  config_.valid_only_filter = this->get_parameter("spn_filter").as_bool();
  config_.timestamp_mode = this->get_parameter("timestamp_mode").as_int();
  config_.lidar_frame_id = this->get_parameter("frame_id").as_string();
  config_.point_format = static_cast<PointFormat>(this->get_parameter("point_format").as_int());
}

void VoyantSensorDriver::initialize()
{
  RCLCPP_INFO(get_logger(), "[+] Initializing Voyant Sensor Driver");
  VoyantClient::setupSignalHandling();

  // Try to connect to the sensor
  try
  {
    RCLCPP_INFO(get_logger(), "[+] Connecting to sensor: %s", config_.binding_address.c_str());
    RCLCPP_INFO(get_logger(),
                "[+] Using point format: %s",
                pointFormatToString(config_.point_format).c_str());
    client_ = std::make_shared<VoyantClient>(config_.binding_address,
                                             config_.multicast_group,
                                             config_.interface_address);

    // Check if the client is connected
    if(!client_->isValid())
    {
      RCLCPP_ERROR(get_logger(), "[-] Failed to initialize the Voyant Sensor Driver");
      rclcpp::shutdown();
    }
    while(!VoyantClient::isTerminated())
    {
      if(client_->tryReceiveNextFrame())
      {
        VoyantFrameWrapper &frame = client_->latestFrame();
        const VoyantHeaderWrapper header_msg = frame.header();
        RCLCPP_INFO(get_logger(), "[+] Connected to sensor: %s", header_msg.deviceId().c_str());

        return; // Successful connection
      }
    }
    throw std::runtime_error("[-] Sensor connection failed");
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "[-] Connection failed: %s", e.what());
  }
}

sensor_msgs::msg::PointCloud2 VoyantSensorDriver::pointDatatoRosMsg(VoyantFrameWrapper &frame)
{
  return convertFrameByFormat(frame, config_);
}

void VoyantSensorDriver::publishPointCloud()
{
  bool published_metadata = false;
  while(rclcpp::ok() && !client_->isTerminated())
  {
    bool frame_received = false;
    try
    {
      if(client_->tryReceiveNextFrame())
      {
        VoyantFrameWrapper &frame = client_->latestFrame();
        sensor_msgs::msg::PointCloud2 cloud_msg = this->pointDatatoRosMsg(frame);
        points_pub->publish(cloud_msg);
        frame_received = true;
        if(!published_metadata)
        {
          // Create and publish static metadata message
          voyant_ros::msg::VoyantDeviceMetadata metadata;
          metadata.header.stamp = this->now();
          metadata.header.frame_id = config_.lidar_frame_id;
          metadata.device_id = frame.header().deviceId();
          metadata.proto_version_hash = frame.header().protoVersion().toU32Hash();
          metadata.api_version_hash = frame.header().apiVersion().toU32Hash();
          metadata.firmware_version_hash = frame.header().firmwareVersion().toU32Hash();
          metadata.hdl_version_hash = frame.header().hdlVersion().toU32Hash();

          metadata_pub->publish(metadata);
          RCLCPP_INFO(get_logger(),
                      "Published static metadata for device: %s",
                      metadata.device_id.c_str());
          published_metadata = true;
        }
      }
    }
    catch(const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "[-] Error: %s", e.what());
      rclcpp::shutdown();
    }
    // sleep for some time to avoid busy looping, but sleep less when we are actively receiving frames
    std::this_thread::sleep_for(std::chrono::milliseconds(frame_received ? 1 : 10));
  }
}

} // namespace voyant_ros
