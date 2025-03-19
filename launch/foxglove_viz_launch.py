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
