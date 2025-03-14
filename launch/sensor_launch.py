# Copyright (c) 2024-2025 Voyant Photonics, Inc.
#
# This example code is licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    sensor_cfg_yaml_path = os.path.join(
        get_package_share_directory("voyant-ros"), "config", "sensor_params.yaml"
    )

    # LiDAR Node
    lidar_node = Node(
        package="voyant-ros",
        executable="voyant_sensor_node",
        output="screen",
        parameters=[sensor_cfg_yaml_path],
    )

    # Foxglove Studio
    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    foxglove_bridge = ExecuteProcess(
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]
    )

    return LaunchDescription([lidar_node, foxglove_bridge, foxglove_studio])
