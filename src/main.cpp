// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant_ros/sensor_driver.hpp"
#include <cbindings/logging_utils_ffi.hpp>

int main(int argc, char **argv)
{
  voyant_log_init_c();
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<voyant_ros::VoyantSensorDriver>();
    rclcpp::spin(node);
  }
  catch(const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("voyant_sensor_node"), "[-] Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
