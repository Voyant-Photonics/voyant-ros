// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant-ros/sensor_driver.hpp"
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>

VoyantSensorDriver::VoyantSensorDriver()
    : Node("voyant_sensor_node")
{
  getParams();
  initialize();
  points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("voyant_points", 100);
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
  this->declare_parameter<std::string>("timestamp_mode", "TIME_FROM_ROS");
  this->declare_parameter<std::string>("frame_id", "lidar_sensor");

  binding_address_ = this->get_parameter("binding_address").as_string();
  multicast_group_ = this->get_parameter("multicast_group").as_string();
  interface_address_ = this->get_parameter("interface_address").as_string();
  valid_only_filter_ = this->get_parameter("spn_filter").as_bool();
  timestamp_mode_ = this->get_parameter("timestamp_mode").as_string();
  lidar_frame_id_ = this->get_parameter("frame_id").as_string();
}

void VoyantSensorDriver::initialize()
{
  RCLCPP_INFO(get_logger(), "[+] Initializing Voyant Sensor Driver");
  VoyantClient::setupSignalHandling();

  // Try to connect to the sensor
  try
  {
    RCLCPP_INFO(get_logger(), "[+] Connecting to sensor: %s", binding_address_.c_str());
    client_ = std::make_shared<VoyantClient>(binding_address_, multicast_group_, interface_address_);

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

  pcl::PointCloud<VoyantPoint> pcl_cloud;
  const auto &points = frame.points();

  // Reserve space for the point cloud
  const size_t point_count = points.size();
  pcl_cloud.resize(point_count);
  // An ordered pointcloud has height 1 and width equal to the number of points
  pcl_cloud.width = point_count;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  size_t index = 0;

  // convert points to pcl point cloud
  for(const auto &p : points)
  {
    // Only include valid points
    if(valid_only_filter_ && p.drop_reason() != DropReason::SUCCESS)
    {
      continue;
    }

    VoyantPoint &point = pcl_cloud.points[index]; // Overwrite the point, this is safe because we
                                                  // reserved the space and we don't have to use
                                                  // push_back
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.v = p.radial_vel();
    point.snr = p.snr_linear();
    point.drop_reason = static_cast<uint8_t>(p.drop_reason());
    point.timestamp_nsecs = p.timestamp_nanosecs();
    point.point_idx = p.point_index();

    index++; // Next valid slot
  }

  pcl_cloud.resize(index);

  // Convert pcl cloud to ros message
  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  // set header timestamps
  if(timestamp_mode_ == "TIME_FROM_SENSOR")
  {
    ros_cloud.header.stamp.sec = frame.header().timestampSeconds();
    ros_cloud.header.stamp.nanosec = frame.header().timestampNanoseconds();
  }
  else
  {
    auto now = this->now();
    ros_cloud.header.stamp = now;
  }
  ros_cloud.header.frame_id = lidar_frame_id_;
  return ros_cloud;
}

void VoyantSensorDriver::publishPointCloud()
{
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
