# Official ROS drivers for Voyant Lidars

This ROS package provides support for Voyant sensors targeting ROS2 Humble distro. Configure the sensor aka `client` address using the `config/sensor_params.yaml` file.
Follow the official [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2 Humble distro on your system.

## Supported device
- Medowlark: [Specsheet](https://voyantphotonics.com/products/)

## Installation
1. Clone the repository
```bash
git clone git@github.com:Voyant-Photonics/voyant-ros.git
```
2. Install the package dependencies

   - The pointcloud visualization of this package also supports `foxglove` package. Install foxglove from the official website [here](https://foxglove.dev/download/).

        Along with the foxglove, install the `ros2-foxglove` bridge.
        ```bash
        sudo apt install ros-humble-foxglove-*
        ```

   - Install the ROS dependencies using the rosdep command

        ```bash
        sudo apt install ros-humble-pcl-ros \
                         ros-humble-rviz2
        ```
        > **Note**
        > You can also install it using the rosdep command
        > ```bash
        > rosdep install --from-paths src --ignore-src -r -y
        > ```
        > The pointcloud can also be visualized using the `rviz`, set `use_rviz` launch argument `true` to visualize pointcloud in `rviz`

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
or
```bash
ros2 launch voyant-ros sensor_launch.py use_rviz:=true # for rviz
```


## Configure the Foxglove GUI for pointcloud visualization
The above command will launch the driver and start publishing the pointcloud data on the `/voyant_points` topic. This will also open the foxglove GUI to visualize the pointcloud data.
1. Connect the foxglove to the default `ws://localhost:8765` websocket server.
2. On the top right corner in the foxglove GUI title bar, click on the `layout` button and import the config file from the `config/foxglove_layout.json` file.
This will load the layout with the pointcloud data, where you can visualize the pointcloud data in 3 different color maps.

Read the [Foxglove Colormap Documentation](https://github.com/Voyant-Photonics/voyant-api/tree/main/crates/foxglove_publisher/README.md) for more information on the colormap.
