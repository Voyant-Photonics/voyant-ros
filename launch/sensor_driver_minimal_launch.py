# Copyright (c) 2024-2025 Voyant Photonics, Inc.
#
# This example code is licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


# Minimal launch file for the sensor driver
def generate_launch_description():

    voyant_ros_pkg_path = get_package_share_directory("voyant-ros")
    sensor_cfg_yaml_path = os.path.join(
        voyant_ros_pkg_path, "config", "sensor_params.yaml"
    )

    # LiDAR Node
    lidar_node = Node(
        package="voyant-ros",
        executable="voyant_sensor_node",
        output="screen",
        parameters=[sensor_cfg_yaml_path],
    )

    return LaunchDescription([lidar_node])
