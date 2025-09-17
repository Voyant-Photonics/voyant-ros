// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include <string>

namespace voyant_ros
{

enum class TimestampMode
{
  // TODO: Breaking change - Add TimestampMode::UNKNOWN = 0
  TIME_FROM_SENSOR = 0,
  TIME_FROM_ROS = 1,
};

enum class PointFormat
{
  UNKNOWN = 0,
  MDL_STANDARD = 1,
  MDL_EXTENDED = 2,
};

/**
 * @brief Sensor configuration parameters
 */
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
  PointFormat point_format = PointFormat::UNKNOWN;
};

/**
 * @brief Convert PointFormat enum to string representation
 * @param format The point format enum value
 * @return String representation of the format
 */
inline std::string pointFormatToString(PointFormat format)
{
  switch(format)
  {
    case PointFormat::MDL_STANDARD:
      return "MDL_STANDARD";
    case PointFormat::MDL_EXTENDED:
      return "MDL_EXTENDED";
    case PointFormat::UNKNOWN:
      return "UNKNOWN";
    default:
      return "INVALID(" + std::to_string(static_cast<int>(format)) + ")";
  }
}

} // namespace voyant_ros
