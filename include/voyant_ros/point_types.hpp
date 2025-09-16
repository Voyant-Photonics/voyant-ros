// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace voyant_ros
{

/**
 * @brief Point structure for the Voyant LiDAR sensor
 */
struct EIGEN_ALIGN16 VoyantPoint
{
  PCL_ADD_POINT4D; // This adds x, y, z, and padding
  float v;         // Radial velocity
  float snr;       // Signal-to-noise ratio
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

} // namespace voyant_ros

// Register point types with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
    voyant_ros::VoyantPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, v, v)(float, snr, snr)(
        uint8_t,
        drop_reason,
        drop_reason)(int32_t, timestamp_nsecs, timestamp_nsecs)(uint32_t, point_idx, point_idx))
