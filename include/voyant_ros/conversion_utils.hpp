// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include "voyant_ros/msg/voyant_device_metadata.hpp"
#include "voyant_ros/point_types.hpp"
#include "voyant_ros/sensor_params.hpp"
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <voyant_frame.hpp>
#include <voyant_version.hpp>

namespace voyant_ros
{

/// Device-metadata topic as it appears in a bag. The driver publishes the relative
/// name, which resolves to this outside a namespace; readers match the trailing path
/// segment so a namespaced capture still resolves to it.
inline constexpr const char *kDeviceMetadataTopic = "/device_metadata";

/**
 * @brief Fill one PCL point from frame data
 */
inline void fillPointFromFrame(VoyantPoint &point,
                               const PointData &p,
                               const VoyantVec3 &xyz,
                               const VoyantFrame &frame)
{
  // A point dropped before a range was measured keeps its real angles but has no
  // x/y/z representation, so publish it as NaN -- the absent return that
  // is_dense = false advertises -- rather than a position it never had.
  if(p.range_m > 0.0f)
  {
    point.x = xyz.x;
    point.y = xyz.y;
    point.z = xyz.z;
  }
  else
  {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
  }
  point.v = p.doppler_mps;
  point.snr = p.snr;
  point.drop_reason = p.drop_reason;
  point.timestamp_nsecs = p.timestamp_nanosecs;
  point.azimuth_idx = p.azimuth_idx;
  point.elevation_idx = p.elevation_idx;
  point.calibrated_reflectance = p.calibrated_reflectance;
  point.frame_index = frame.frameIndex();
}

/**
 * @brief Frame to PointCloud2 converter
 *
 * Frames contain invalid returns only when the source keeps them (client
 * diagnostic mode / playback keep_invalid_points), so every point is published.
 * Per-point timestamps are offsets from the frame stamp in the cloud header.
 */
inline sensor_msgs::msg::PointCloud2 convertFrameToPointCloud2(const VoyantFrame &frame,
                                                               const SensorParams &config)
{
  const std::vector<PointData> &points = frame.points();
  const std::vector<VoyantVec3> xyz = frame.xyz(); // index-aligned with points()

  pcl::PointCloud<VoyantPoint> pcl_cloud;
  pcl_cloud.resize(points.size());
  pcl_cloud.width = points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = false;

  for(size_t i = 0; i < points.size(); i++)
  {
    fillPointFromFrame(pcl_cloud.points[i], points[i], xyz[i], frame);
  }

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);

  switch(static_cast<TimestampMode>(config.timestamp_mode))
  {
    case TimestampMode::TIME_FROM_SENSOR:
      ros_cloud.header.stamp.sec = static_cast<int32_t>(frame.timestampSeconds());
      ros_cloud.header.stamp.nanosec = static_cast<uint32_t>(frame.timestampNanoseconds());
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

/**
 * @brief Build the device metadata message that accompanies a point cloud stream
 *
 * The API and interface-contract versions describe the linked library; everything
 * else is the sensor's own and comes from the frame's state, so it stays unset for a
 * frame that carries none. product_id and serial_number are device_id as data.
 */
inline voyant_ros::msg::VoyantDeviceMetadata deviceMetadataFromFrame(const VoyantFrame &frame,
                                                                     const SensorParams &config)
{
  voyant_ros::msg::VoyantDeviceMetadata metadata;
  metadata.header.frame_id = config.lidar_frame_id;
  metadata.device_id = frame.deviceId();
  metadata.api_version = voyantApiVersion();
  metadata.interface_contract_version = voyantInterfaceContractVersion();

  if(std::optional<SensorState> state = frame.sensorState())
  {
    const DeviceInfo &device = state->device;
    metadata.product_id = static_cast<uint8_t>(device.product_id);
    metadata.serial_number = device.serial_number;
    metadata.firmware_version = std::to_string(device.mcu_version_major) + "." +
                                std::to_string(device.mcu_version_minor) + "." +
                                std::to_string(device.mcu_version_patch);
    metadata.hdl_version = std::to_string(device.fpga_version_major) + "." +
                           std::to_string(device.fpga_version_minor) + "." +
                           std::to_string(device.fpga_version_patch);
  }

  return metadata;
}

/**
 * @brief Convert PointCloud2 back to a recordable VoyantFrame
 *
 * The rebuilt frame keeps the cloud's timeline and the sensor's identity, but declares
 * no state: a bag carries the points, not the per-frame heartbeat behind them, so the
 * recording reports "none was recorded" rather than defaults that read as measurements.
 * combine_method and user_data are internal-only, so the cloud has no field for
 * them and they rebuild as zero; so do the angles of a point published as an absent
 * return, which has no x/y/z to invert (its scan cell survives in the index fields).
 */
inline VoyantFrame convertPointCloud2ToFrame(const sensor_msgs::msg::PointCloud2 &cloud,
                                             const voyant_ros::msg::VoyantDeviceMetadata &metadata)
{
  // Convert ROS PointCloud2 to PCL
  pcl::PointCloud<VoyantPoint> pcl_cloud;
  pcl::fromROSMsg(cloud, pcl_cloud);

  uint32_t frame_index = 0;
  if(!pcl_cloud.empty())
  {
    // We will confirm that all points have the same frame index
    frame_index = pcl_cloud.points[0].frame_index;
  }
  else
  {
    std::cerr << "Attempting conversion of empty PointCloud2 into VoyantFrame" << std::endl;
  }

  std::vector<PointData> points;
  std::vector<VoyantVec3> positions;
  points.reserve(pcl_cloud.size());
  positions.reserve(pcl_cloud.size());

  for(const auto &pcl_point : pcl_cloud.points)
  {
    if(pcl_point.frame_index != frame_index)
    {
      std::cerr << "Warning: Skipping point with inconsistent frame_index. Expected: " << frame_index
                << ", Found: " << pcl_point.frame_index << std::endl;
      continue;
    }

    PointData p{};
    p.doppler_mps = pcl_point.v;
    p.snr = pcl_point.snr;
    p.calibrated_reflectance = pcl_point.calibrated_reflectance;
    p.timestamp_nanosecs = pcl_point.timestamp_nsecs;
    p.azimuth_idx = pcl_point.azimuth_idx;
    p.elevation_idx = pcl_point.elevation_idx;
    p.drop_reason = pcl_point.drop_reason;

    points.push_back(p);
    positions.push_back(VoyantVec3{pcl_point.x, pcl_point.y, pcl_point.z});
  }

  // The API owns the projection in both directions; a NaN position -- how this driver
  // publishes an absent return -- rebuilds as range 0, the same state it was read from.
  if(!setPointsXyz(points, positions))
  {
    throw std::runtime_error(
        "Failed to restore point geometry from PointCloud2 (device: " + metadata.device_id + ")");
  }

  VoyantFrame::StatelessFrameDesc desc;
  desc.source = static_cast<ProductId>(metadata.product_id);
  desc.serialNumber = metadata.serial_number;
  desc.timestampSeconds = cloud.header.stamp.sec;
  desc.timestampNanoseconds = static_cast<int32_t>(cloud.header.stamp.nanosec);
  desc.frameIndex = frame_index;

  std::optional<VoyantFrame> frame = VoyantFrame::stateless(std::move(points), desc);
  if(!frame)
  {
    throw std::runtime_error(
        "Failed to rebuild a frame from PointCloud2 (device: " + metadata.device_id + ")");
  }
  return std::move(*frame);
}

} // namespace voyant_ros
