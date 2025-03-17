# Official ROS drivers for Voyant Lidars

This ROS package provides support for Voyant sensors targeting ROS2 Humble distro. Configure the sensor aka `client` address using the `config/sensor_params.yaml` file.
Follow the official [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2 Humble distro on your system.

## Supported device
- Device: Specsheet

## Installation
1. Clone the repository
```bash
git clone git@github.com:Voyant-Photonics/voyant-ros.git
```
2. Install the package dependencies

   - The pointcloud visualization of this package also supports `foxglove` package. Install foxglove from the official website [here](https://foxglove.dev/download/).

        Along with the foxglove, install the `ros2-foxglove` bridge.
        ```bash
        sudo apt install ros-humble-foxglove-bridge
        ```

   - Install the ROS dependencies using the rosdep command

        ```bash
        sudo apt install ros-humble-pcl-ros
        ```
        > Note:
        > You can also install it using the rosdep command
        > ```bash
        > rosdep install --from-paths src --ignore-src -r -y
        > ```
        > The pointcloud can also be vizable using the `rviz`.

   - Install Voyant API

        Follow the native installation guide from the official [Voyant SDK repo](https://github.com/Voyant-Photonics/voyant-sdk#native-installation)
3. Build the package
```bash
colcon build --symlink-install --packages-select voyant-ros
```

## Running the package
1. Source the ROS2 workspace
```bash
source install/setup.bash
```
2. Launch the driver
```bash
ros2 launch voyant-ros sensor_launch.py
```

## Configure the Foxglove GUI for pointcloud visualization
The above command will launch the driver and start publishing the pointcloud data on the `/voyant_points` topic. This will also open the foxglove GUI to visualize the pointcloud data.
Connect the foxglove to the default `ws://localhost:8765` websocket server.
