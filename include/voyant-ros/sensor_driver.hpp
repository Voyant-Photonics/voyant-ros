// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include <chrono>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voyant_client.hpp>

/**
 * @brief Point structure for the Voyant LiDAR sensor
 *  These structures are shared with the bin_to_mcap conversion.
 */
struct EIGEN_ALIGN16 VoyantPoint
{
  PCL_ADD_POINT4D; // This adds x, y, z, and padding
  float v;
  float snr;
  uint8_t drop_reason;
  int32_t timestamp_nsecs;
  uint32_t point_idx;

  inline VoyantPoint()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f; // Set padding to 1 to prevent undefined behavior.
    v = 0.0f;
    snr = 0.0f;
    drop_reason = 0;
    timestamp_nsecs = 0;
    point_idx = 0;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VoyantPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, v, v)(float, snr, snr)(uint8_t, drop_reason, drop_reason))

enum class TimestampMode
{
  TIME_FROM_SENSOR = 0,
  TIME_FROM_ROS = 1,
};

struct SensorParams
{
  std::string mcap_output;
  std::string bin_input;
  std::string lidar_frame_id;
  int timestamp_mode;
  bool valid_only_filter;
  std::string storage_id;
  std::string serialization_format;
  std::string topic_name;
  std::string binding_address;
  std::string multicast_group;
  std::string interface_address;
};

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

  // Voyant client
  std::shared_ptr<VoyantClient> client_;

  // Sensor config
  SensorParams config_;
};

template <typename PointT, typename FrameT, typename ConfigT>
sensor_msgs::msg::PointCloud2 convertFrameToPointCloud2(const FrameT &frame, const ConfigT &config)
{
  pcl::PointCloud<PointT> pcl_cloud;
  const auto &points = frame.points();
  size_t point_count = points.size();

  pcl_cloud.resize(point_count);
  pcl_cloud.width = point_count;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  size_t index = 0;
  for(const auto &p : points)
  {
    if(config.valid_only_filter && p.drop_reason() != DropReason::SUCCESS)
    {
      continue;
    }

    PointT &point = pcl_cloud.points[index++];
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.v = p.radial_vel();
    point.snr = p.snr_linear();
    point.drop_reason = static_cast<uint8_t>(p.drop_reason());
    point.timestamp_nsecs = p.timestamp_nanosecs();
    point.point_idx = p.point_index();
  }
  pcl_cloud.resize(index);

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  switch(static_cast<TimestampMode>(config.timestamp_mode))
  {
    case TimestampMode::TIME_FROM_SENSOR:
      ros_cloud.header.stamp.sec = frame.header().timestampSeconds();
      ros_cloud.header.stamp.nanosec = frame.header().timestampNanoseconds();
      break;

    case TimestampMode::TIME_FROM_ROS:
      ros_cloud.header.stamp = rclcpp::Clock().now();
      break;

    default:
      throw std::runtime_error(
          "Unknown timestamp_mode enum value: " + std::to_string(config.timestamp_mode));
  }

  ros_cloud.header.frame_id = config.lidar_frame_id;
  return ros_cloud;
}
