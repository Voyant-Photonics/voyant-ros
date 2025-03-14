from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess


def generate_launch_description():
    current_pkg = FindPackageShare("voyant-ros")

    # Load params
    sensor_cfg_yaml_path = PathJoinSubstitution(
        [current_pkg, "config", "sensor_params.yaml"]
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
