// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant-ros/sensor_driver.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoyantSensorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
