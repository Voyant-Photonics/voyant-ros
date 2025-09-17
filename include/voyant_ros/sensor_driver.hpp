// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/conversion_utils.hpp"
#include "voyant_ros/msg/voyant_device_metadata.hpp"
#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voyant_client.hpp>

namespace voyant_ros
{

/**
 * @class VoyantSensorDriver
 * @brief ROS2 node for the Voyant LiDAR sensor
 */
class VoyantSensorDriver : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Voyant Sensor Driver object
   */
  VoyantSensorDriver();

  /**
   * @brief Destroy the Voyant Sensor Driver object
   */
  virtual ~VoyantSensorDriver();

private:
  /**
   * @brief Initialize the connection to the sensor
   */
  void initialize();

  /**
   * @brief Get the parameters from the ROS2 parameter server
   */
  void getParams();

  /**
   * @brief Publish the point cloud to the ROS2 topic
   */
  void publishPointCloud();

  /**
   * @brief Convert the Voyant frame to a ROS2 message
   *
   * @param frame The Voyant frame
   * @return sensor_msgs::msg::PointCloud2 The ROS2 point cloud message
   */
  sensor_msgs::msg::PointCloud2 pointDatatoRosMsg(VoyantFrameWrapper &frame);

  // ROS components
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<voyant_ros::msg::VoyantDeviceMetadata>::SharedPtr metadata_pub;

  // Voyant client
  std::shared_ptr<VoyantClient> client_;

  // Sensor config
  SensorParams config_;
};

} // namespace voyant_ros
