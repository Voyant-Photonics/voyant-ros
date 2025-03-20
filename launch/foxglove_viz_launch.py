# Copyright (c) 2024-2025 Voyant Photonics, Inc.
#
# This example code is licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

# This file is used by `sensor_launch.py` to launch Foxglove Studio and the Foxglove Bridge.
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])
    foxglove_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "foxglove_bridge",
            "foxglove_bridge_launch.xml",
            "max_qos_depth:=100",
        ]
    )

    return LaunchDescription(
        [
            foxglove_studio,
            foxglove_bridge,
        ]
    )
