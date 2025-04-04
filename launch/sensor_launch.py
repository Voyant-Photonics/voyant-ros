# Copyright (c) 2024-2025 Voyant Photonics, Inc.
#
# This example code is licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
import os
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    voyant_ros_pkg_path = get_package_share_directory("voyant-ros")
    sensor_cfg_yaml_path = os.path.join(
        voyant_ros_pkg_path, "config", "sensor_params.yaml"
    )

    # use rviz to visualize the point cloud, if false uses foxglove studio
    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Use RViz for visualization if true, else use Foxglove Studio",
    )

    # LiDAR Node
    lidar_node = Node(
        package="voyant-ros",
        executable="voyant_sensor_node",
        output="screen",
        parameters=[sensor_cfg_yaml_path],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("voyant-ros"), "config", "viz.rviz"
            ),
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # Foxglove Studio
    foxglove_launch_file = os.path.join(
        voyant_ros_pkg_path,
        "launch",
        "foxglove_viz_launch.py",
    )
    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(foxglove_launch_file)]),
        condition=UnlessCondition(use_rviz),
    )

    return LaunchDescription([use_rviz_arg, lidar_node, rviz, foxglove_bridge])
