# Official ROS drivers for Voyant Lidars

[![CI](https://github.com/Voyant-Photonics/voyant-ros/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/Voyant-Photonics/voyant-ros/actions/workflows/docker-image.yml)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-orange)
![voyant-api](https://img.shields.io/badge/voyant--api-%E2%89%A5%200.9.2-green)
[![License](https://img.shields.io/github/license/Voyant-Photonics/voyant-ros)](LICENSE)
[![Latest release](https://img.shields.io/github/v/release/Voyant-Photonics/voyant-ros)](https://github.com/Voyant-Photonics/voyant-ros/releases)

This ROS package provides support for Voyant sensors.
Configure the sensor (client) address using the `config/sensor_params.yaml` file.
Pre-built Debian packages target `ROS2 Humble` and `Ubuntu 22.04`.
`ROS2 Jazzy` and `Ubuntu 24.04` are supported by building from source.
For other OS/distro combinations, refer to [Option 3: Docker](#option-3-docker) below.

## Supported device

- Carbon: [Specsheet](https://voyantphotonics.com/products/) (specsheet coming soon...)
- Meadowlark: [Specsheet](https://voyantphotonics.com/products/) (specsheet coming soon...) — for Meadowlark sensors, please use the previous release [`v0.2.2`](https://github.com/Voyant-Photonics/voyant-ros/releases/v0.2.2) of this package.

## Pre-requisites

Follow the official [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2 Humble on your system.

## Installation

### Option 1: Native — Ubuntu 22.04 + ROS2 Humble (pre-built packages)

#### 1. Install ROS2 Humble

Follow the official [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

#### 2. Install Cap'n Proto

Cap'n Proto is a required runtime dependency that must be built from source. Follow the **Installation: Unix** > **From Release Tarball** instructions at [capnproto.org/install.html](https://capnproto.org/install.html). At the time of writing:

```bash
curl -O https://capnproto.org/capnproto-c++-1.1.0.tar.gz
tar zxf capnproto-c++-1.1.0.tar.gz
cd capnproto-c++-1.1.0
./configure
make -j6 check
sudo make install
sudo ldconfig
cd ..
```

#### 3. Download and install packages

Download the following `.deb` files from the [latest voyant-ros release](https://github.com/Voyant-Photonics/voyant-ros/releases/latest) (the `voyant-api` packages are bundled here alongside the ROS package, version-matched):

- `voyant-api_*_amd64.deb`
- `voyant-api-dev_*_amd64.deb`
- `ros-humble-voyant-ros_*_amd64.deb`

```bash
cd ~/Downloads  # or wherever you saved the .deb files
sudo apt update
sudo apt install -y ./voyant-api*.deb
sudo apt install -y ./ros-humble-voyant-ros*.deb
```

#### 4. [Optional] Install visualization tools

- Install Foxglove Studio from the [official website](https://foxglove.dev/download/), or use [Foxglove Web](https://app.foxglove.dev/) at `ws://localhost:8765`
- Install the ROS2-Foxglove bridge:

  ```bash
  sudo apt install ros-humble-foxglove-*
  ```

---

### Option 2: Native — Ubuntu 24.04 + ROS2 Jazzy (build from source)

#### 1. Install ROS2 Jazzy

Follow the official [ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

<!-- markdownlint-disable-next-line MD024 -->
#### 2. Install Cap'n Proto

Cap'n Proto is a required runtime dependency that must be built from source.
Follow the **Installation: Unix** > **From Release Tarball** instructions at [capnproto.org/install.html](https://capnproto.org/install.html).
At the time of writing:

```bash
curl -O https://capnproto.org/capnproto-c++-1.1.0.tar.gz
tar zxf capnproto-c++-1.1.0.tar.gz
cd capnproto-c++-1.1.0
./configure
make -j6 check
sudo make install
sudo ldconfig
cd ..
```

#### 3. Install Voyant API

Download `voyant-api_*_amd64.deb` and `voyant-api-dev_*_amd64.deb` from the [latest voyant-sdk release](https://github.com/Voyant-Photonics/voyant-sdk/releases/latest) and install them:

```bash
cd ~/Downloads  # or location of downloaded files
sudo apt update
sudo apt install -y ./voyant-api_*$(dpkg --print-architecture).deb \
                    ./voyant-api-dev_*$(dpkg --print-architecture).deb
```

#### 4. Install ROS dependencies

```bash
sudo apt install ros-jazzy-pcl-ros ros-jazzy-rviz2
```

> **Note**
> You can also install dependencies using rosdep:
>
> ```bash
> rosdep install --from-paths src --ignore-src -r -y
> ```
>
> The pointcloud can also be visualized using RViz — set the `use_rviz` launch argument to `true`.

#### 5. Clone and build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:Voyant-Photonics/voyant-ros.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select voyant_ros
```

#### 6. [Optional] Install visualization tools

- Install Foxglove Studio from the [official website](https://foxglove.dev/download/), or use [Foxglove Web](https://app.foxglove.dev/) at `ws://localhost:8765`
- Install the ROS2-Foxglove bridge:

  ```bash
  sudo apt install ros-jazzy-foxglove-*
  ```

---

### Option 3: Docker

Use this for other OS/distro combinations, custom RMW implementations, or isolated environments.
The image has been tested with ROS2 Humble and Jazzy, and RMW implementations FastRTPS and CycloneDDS.

Build from the repo root:

```bash
docker build --build-arg "VIZ_BRIDGE=true" -t voyant_ros2_container .
```

To target a specific ROS distro or RMW implementation:

```bash
docker build --build-arg "VIZ_BRIDGE=true" \
             --build-arg "ROS_DISTRO=jazzy" \ # humble, jazzy
             --build-arg "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \ # rmw_fastrtps_cpp, rmw_cyclonedds_cpp
             -t voyant_ros2_container .
```

> **Note**
> The `VIZ_BRIDGE` argument is optional and can be set to `true` to install the Foxglove bridge for visualization.
The default value is `false`. If the argument is set to `true`, the Foxglove bridge will be installed.
Follow the instructions from the [Visualization Guide](https://voyant-photonics.github.io/foxglove/) to configure Foxglove for pointcloud visualization in a separate terminal or in a web browser.

Run the container:

```bash
docker run -it --network=host voyant_ros2_container
```

## Running the package

> 🚧 **Temporary fix: when using `voyant-api` 0.9.2 or later (Carbon sensor)**
>
> The Carbon client requires the sensor to be brought up via Voyant Visualizer
> before the ROS node can connect. Until this is integrated into
> the driver, follow these steps on every power cycle of the sensor:
>
> 1. Bring up the sensor following the sensor bring-up steps in the guide.
> 2. Open the Voyant visualizer and confirm you see a live pointcloud
>    from the sensor.
> 3. Close the visualizer (it must release the multicast socket before
>    the ROS driver can bind to it).
> 4. Build and run the ROS node using the steps below.

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

## Converting mcap files to bin format

You can only convert MCAP files with the correct data.

> At time of writing, that means you will need to record the `/device_metadata` field and
> to use `point_format: 2 # MDL_EXTENDED` when recording.

**Build the tool:**

```bash
# source ROS2 Humble
source /opt/ros/humble/setup.bash
# build with colcon
colcon build --symlink-install --packages-select voyant_ros
# Source the workspace (required for compiled VoyantDeviceMetadata.msg)
source install/setup.bash
```

Edit the `yaml` config file to pass the correct file paths.

**Run the tool:**

```bash
./build/voyant_ros/bin/voyant_mcap_to_bin config/mcap_to_bin_params.yaml
```

## Configuring Foxglove for Pointcloud Visualization

The launch command will start the driver and publish pointcloud data on the `/point_cloud` topic.
It will also open the Foxglove GUI for visualization.

1. Click on `Open connection...` in the Foxglove GUI on the left panel.
2. Connect Foxglove to the default Foxglove websocket server at `ws://localhost:8765`
3. In the top right corner of the Foxglove GUI title bar, click on the `Layout` button and import the configuration file from `config/voyant_ros_foxglove_cfg.json`

This will load a layout with pointcloud data visualization, offering three different color maps.
For more information on the colormap options, refer to the [Foxglove Colormap Documentation](https://voyant-photonics.github.io/foxglove/).

## Managing the Foxglove Layout Config

The Foxglove configuration uses a template system to keep user scripts version-controlled as separate `.ts` files.

**Files:**

- `config/foxglove_user_scripts/*.ts` - Edit these scripts directly
- `config/voyant_foxglove_cfg.template.json` - Edit layout, panels, and global variables here
- `config/voyant_foxglove_cfg.json` (**Auto-generated, do not edit**) - The layout to import into Foxglove
After modifying any user script or the template, regenerate the config:

```bash
python3 config/build_foxglove_config.py \
    --template config/voyant_foxglove_cfg.template.json \
    --scripts-dir config/foxglove_user_scripts \
    --output config/voyant_foxglove_cfg.json
```

And clean up the formatting:

```bash
pre-commit run --all-files
```
