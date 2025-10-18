# Official ROS drivers for Voyant Lidars

This ROS package provides support for Voyant sensors targeting the ROS2 Humble distribution. Configure the sensor (client) address using the `config/sensor_params.yaml` file. This package only supports `ROS2 Humble` and `Ubuntu 22.04` for now, and it is not guaranteed to work with other ROS2 distributions or operating systems. Support for other distributions and operating systems will be added in the future.

For Docker instructions on other ROS2 distributions and RMW implementations, refer to the [Docker Instructions](#docker-instructions) section.

## Supported device

- Meadowlark: [Specsheet](https://voyantphotonics.com/products/) (specsheet coming soon...)

## Pre-requisites

Follow the official [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2 Humble on your system.

## Installation

### 1. Clone the repository

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:Voyant-Photonics/voyant-ros.git
```

### 2. Install the package dependencies

#### Visualization Tools
- For Foxglove visualization, install Foxglove Studio from the [official website](https://foxglove.dev/download/)
- Alternatively you can also visualize the pointcloud over the Web using the Foxglove Web. Simply open the [Foxglove Web](https://app.foxglove.dev/) and connect to the default Foxglove websocket server at `ws://localhost:8765`
- Install the ROS2-Foxglove bridge:
  ```bash
  sudo apt install ros-humble-foxglove-*
  ```

#### ROS Dependencies
```bash
sudo apt install ros-humble-pcl-ros ros-humble-rviz2
```
> **Note**
> You can also install it using the rosdep command
> ```bash
> rosdep install --from-paths src --ignore-src -r -y
> ```
> The pointcloud can also be visualized using the RViz, set `use_rviz` launch argument `true` to visualize pointcloud in RViz.

#### Docker Instructions
This repository also provides a Dockerfile to build a Docker image with the ROS2 Humble distribution and the Voyant ROS package.
To build the Docker image, run the following command from the repo root:

```bash
docker build --build-arg "VIZ_BRIDGE=true" -t voyant_ros2_container .
```

The Docker image has been tested on Ubuntu 22.04 and ROS2 distributions like Humble, Iron, Rolling and Jazzy. RMW implementations like FastRTPS and CycloneDDS have been tested with the Docker image.

```bash
docker build --build-arg "VIZ_BRIDGE=true" \
             --build-arg "ROS_DISTRO=rolling" \ # humble, iron, rolling, jazzy
             --build-arg "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \ # rmw_fastrtps_cpp, rmw_cyclonedds_cpp
             -t voyant_ros2_container .
```

> **Note**
> The `VIZ_BRIDGE` argument is optional and can be set to `true` to install the Foxglove bridge for visualization. The default value is `false`. If the argument is set to `true`, the Foxglove bridge will be installed. Follow the instructions from the [Visualization Guide](https://voyant-photonics.github.io/getting-started/visualization.html#importing-configuration-files) to configure Foxglove for pointcloud visualization in separate terminal or in a web browser.

To run the Docker container, execute the following command:

```bash
docker run -it --network=host voyant_ros2_container
```

#### Install Voyant API
Follow the native installation guide from the official [Voyant SDK Documentation](https://voyant-photonics.github.io/getting-started/installation.html).

### 3. Build the package

```bash
source /opt/ros/humble/setup.bash # source ROS2 Humble
colcon build --symlink-install --packages-select voyant_ros
```

## Running the package

### 1. Source the ROS2 workspace
```bash
source install/setup.bash
```

### 2. Launch the driver
```bash
ros2 launch voyant_ros sensor_launch.py
```
or with RViz visualization

```bash
ros2 launch voyant_ros sensor_launch.py use_rviz:=true # for rviz
```

## Converting `.bin` files to ROS2 bag format
The configurations for ROS2 bag can be found in `config/sensor_params.yaml` file. There are two ways you can run use the conversion tool.

### 1. Using the binaries from `colcon build`

If you have already build the `voyant_ros` package using `colcon`, use the binary files as below.
```bash
cd ~/ros2_ws/build/voyant-ros
./bin/voyant_bin_to_mcap ros2_ws/src/voyant-ros/config/sensor_params.yaml # path to your params yaml file
```

### 2. Build the package using `cmake`

```bash
cd ~/ros2_ws/src/voyant-ros
mkdir -p build
cd build
cmake ..
make
./bin/voyant_bin_to_mcap ros2_ws/src/voyant-ros/config/sensor_params.yaml # path to your params yaml file
```

## Configuring Foxglove for Pointcloud Visualization

The launch command will start the driver and publish pointcloud data on the `/point_cloud` topic. It will also open the Foxglove GUI for visualization.

1. Click on `Open connection...` in the Foxglove GUI on the left panel.
2. Connect Foxglove to the default Foxglove websocket server at `ws://localhost:8765`
3. In the top right corner of the Foxglove GUI title bar, click on the `Layout` button and import the configuration file from `config/voyant_ros_foxglove_cfg.json`

This will load a layout with pointcloud data visualization, offering three different color maps. For more information on the colormap options, refer to the [Foxglove Colormap Documentation](https://voyant-photonics.github.io/02_getting-started/visualization.html).
