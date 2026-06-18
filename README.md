# Official ROS2 drivers for Voyant LiDARs

[![CI](https://github.com/Voyant-Photonics/voyant-ros/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/Voyant-Photonics/voyant-ros/actions/workflows/docker-image.yml)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Lyrical-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04%20%7C%2026.04-orange)
![voyant-api](https://img.shields.io/badge/voyant--api-%E2%89%A5%200.9.2-green)
[![License](https://img.shields.io/github/license/Voyant-Photonics/voyant-ros)](LICENSE)
[![Latest release](https://img.shields.io/github/v/release/Voyant-Photonics/voyant-ros)](https://github.com/Voyant-Photonics/voyant-ros/releases)

This ROS2 package provides support for Voyant sensors.
Configure the sensor (client) address using the `config/sensor_params.yaml` file.
It requires [`voyant-api`](https://github.com/Voyant-Photonics/voyant-sdk) **≥ 0.9.2**
(the Carbon baseline). Pre-built releases bundle the version-matched `voyant-api`
packages, so you only install `voyant-api` yourself when building from source —
any release ≥ 0.9.2 works there.
Pre-built Debian packages are published for `ROS2 Humble` (Ubuntu 22.04),
`ROS2 Jazzy` (Ubuntu 24.04), `ROS2 Kilted` (Ubuntu 24.04), and `ROS2 Lyrical`
(Ubuntu 26.04). Building from source is also supported.
For other OS/distro combinations, refer to [Option 3: Docker](#option-3-docker) below.

## Supported device

- Carbon: [Specsheet](https://voyantphotonics.com/products/) (specsheet coming soon...)
- Meadowlark: [Specsheet](https://voyantphotonics.com/products/) (specsheet coming soon...) — for Meadowlark sensors, please use the previous release [`v0.2.2`](https://github.com/Voyant-Photonics/voyant-ros/releases/v0.2.2) of this package.

## Pre-requisites

Install ROS2 for your target platform:

- Ubuntu 22.04: [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ubuntu 24.04: [ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Ubuntu 24.04: [ROS2 Kilted installation guide](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
- Ubuntu 26.04: [ROS2 Lyrical installation guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html)

## Installation

### Option 1: Native — pre-built packages (recommended)

> Pre-built `.deb` packages are published for **Ubuntu 22.04 / ROS2 Humble**,
> **Ubuntu 24.04 / ROS2 Jazzy**, **Ubuntu 24.04 / ROS2 Kilted**, and
> **Ubuntu 26.04 / ROS2 Lyrical**. The commands below are distro-agnostic: they
> use the `$ROS_DISTRO` environment variable, which is set automatically once you
> source your ROS 2 installation (`source /opt/ros/<distro>/setup.bash`). If you
> haven't sourced it yet in this shell, set it explicitly first, e.g.:
>
> ```bash
> export ROS_DISTRO=jazzy   # one of: humble, jazzy, kilted, lyrical
> ```

#### 1. Install ROS2

Follow the official installation guide for your platform:
[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
(Ubuntu 22.04), [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
(Ubuntu 24.04), [ROS2 Kilted](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
(Ubuntu 24.04), or [ROS2 Lyrical](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html)
(Ubuntu 26.04).

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

Download the following `.deb` files from the [latest voyant-ros release](https://github.com/Voyant-Photonics/voyant-ros/releases/latest) (the `voyant-api` packages are bundled here alongside the ROS2 package, version-matched):

- `voyant-api_*_amd64.deb`
- `voyant-api-dev_*_amd64.deb`
- `ros-${ROS_DISTRO}-voyant-ros_*_amd64.deb` — the package matching your distro:
  `ros-humble-…` (Ubuntu 22.04), `ros-jazzy-…` (Ubuntu 24.04), `ros-kilted-…`
  (Ubuntu 24.04), or `ros-lyrical-…` (Ubuntu 26.04)

```bash
cd ~/Downloads  # or wherever you saved the .deb files
sudo apt update
sudo apt install -y ./voyant-api*.deb
# $ROS_DISTRO is set by sourcing /opt/ros/<distro>/setup.bash (or export it, see above)
sudo apt install -y ./ros-${ROS_DISTRO}-voyant-ros*.deb
```

#### 4. [Optional] Install visualization tools

- Install Foxglove Studio from the [official website](https://foxglove.dev/download/), or use [Foxglove Web](https://app.foxglove.dev/) at `ws://localhost:8765`
- Install the ROS2-Foxglove bridge:

  ```bash
  sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
  ```

---

### Option 2: Native — build from source

> Building from source is the right path when you're:
>
> - **developing or modifying** the driver itself,
> - on a **distro/OS combination without a pre-built package** (Option 1 ships
>   Humble, Jazzy, Kilted, and Lyrical only),
> - tracking a **development or pre-release `voyant-api`** rather than a tagged
>   release, or
> - testing **unreleased changes** ahead of the next tagged release.
>
> Otherwise [Option 1](#option-1-native--pre-built-packages-recommended) is the
> simpler path.
>
> The steps below are distro-agnostic via `$ROS_DISTRO`, which is set once you
> source your ROS 2 installation. If it isn't set yet in this shell, export it
> first, e.g. `export ROS_DISTRO=jazzy`.

<!-- markdownlint-disable-next-line MD024 -->
#### 1. Install ROS2

Follow the official installation guide for your distro (see
[Pre-requisites](#pre-requisites) above for the per-distro links), e.g. the
[ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

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
sudo apt install -y ./voyant-api_*amd64.deb \
                    ./voyant-api-dev_*amd64.deb
```

#### 4. Install ROS dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-rviz2
```

#### 5. Clone and build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Voyant-Photonics/voyant-ros.git
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select voyant_ros
```

> **Note**
> You can also install ROS dependencies via rosdep instead of step 4 (run from workspace root after cloning):
>
> ```bash
> rosdep install --from-paths src --ignore-src -r -y
> ```
>
> The pointcloud can also be visualized using RViz — set the `use_rviz` launch argument to `true`.

#### 6. [Optional] Install visualization tools

- Install Foxglove Studio from the [official website](https://foxglove.dev/download/), or use [Foxglove Web](https://app.foxglove.dev/) at `ws://localhost:8765`
- Install the ROS2-Foxglove bridge:

  ```bash
  sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
  ```

---

### Option 3: Docker

Use this for other OS/distro combinations, custom RMW implementations, or isolated environments.
The image has been tested with ROS2 Humble, Jazzy, Kilted, and Lyrical, and RMW implementations FastRTPS and CycloneDDS.

Build from the repo root:

```bash
docker build --build-arg "VIZ_BRIDGE=true" -t voyant_ros2_container .
```

To target a specific ROS distro or RMW implementation (`ROS_DISTRO`: `humble`, `jazzy`, `kilted`, or `lyrical`; `RMW_IMPLEMENTATION`: `rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`):

```bash
docker build --build-arg "VIZ_BRIDGE=true" \
             --build-arg "ROS_DISTRO=jazzy" \
             --build-arg "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
             -t voyant_ros2_container .
```

> **Note**
> The `VIZ_BRIDGE` argument is optional and can be set to `true` to install the Foxglove bridge for visualization. The default value is `false`. Follow the instructions from the [Visualization Guide](https://voyant-photonics.github.io/foxglove/) to configure Foxglove for pointcloud visualization in a separate terminal or in a web browser.

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
> 3. Close the visualizer (it must release the UDP socket before
>    the ROS driver can bind to it).
> 4. Build and run the ROS node using the steps below.

### 1. Source the ROS2 workspace

**If installed from pre-built packages (Option 1):**

```bash
source /opt/ros/<distro>/setup.bash   # e.g. humble, jazzy, kilted, lyrical
```

**If built from source (Option 2):**

```bash
source ~/ros2_ws/install/setup.bash
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

After building `voyant_ros` with `colcon` from the workspace root (`~/ros2_ws`),
set `bin_input` and `mcap_output` in `config/sensor_params.yaml` (both are empty
by default), then source the environment and run the converter. Note the build
directory is `build/voyant_ros` (package name, underscore), and the converters
live in its `bin/` subdirectory.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash   # or your ROS 2 distro
source install/setup.bash           # required for the VoyantDeviceMetadata message
./build/voyant_ros/bin/voyant_bin_to_mcap src/voyant-ros/config/sensor_params.yaml
```

### 2. Build the package using `cmake`

```bash
cd ~/ros2_ws/src/voyant-ros
source /opt/ros/humble/setup.bash   # or your ROS 2 distro
mkdir -p build
cd build
cmake ..
make
./bin/voyant_bin_to_mcap ../config/sensor_params.yaml # path to your params yaml file
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

Edit `config/mcap_to_bin_params.yaml` to set `mcap_input` and `bin_output`
(both are empty by default).

**Run the tool** (from the workspace root, `~/ros2_ws`):

```bash
./build/voyant_ros/bin/voyant_mcap_to_bin src/voyant-ros/config/mcap_to_bin_params.yaml
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

## Releasing (maintainers)

Releases are built and published by the [`Build & Release Debian`](.github/workflows/release-debian.yml)
GitHub Action. It builds the `ros-humble-voyant-ros` (jammy), `ros-jazzy-voyant-ros`
(noble), `ros-kilted-voyant-ros` (noble), and `ros-lyrical-voyant-ros` (resolute)
Debian packages against the matching [`voyant-sdk`](https://github.com/Voyant-Photonics/voyant-sdk/releases)
release (`v<package.xml version>`), smoke-tests installation in a clean container,
and attaches all packages to a GitHub Release. This is separate from the per-PR build in
[`docker-image.yml`](.github/workflows/docker-image.yml) and does **not** run on
every push.

A published release bundles six version-matched `amd64` Debian packages: the
four ROS packages plus `voyant-api_*.deb` and `voyant-api-dev_*.deb` pulled from the
matching `voyant-sdk` release.

### Cut a release (the normal path)

The `<version>` in [`package.xml`](package.xml) is the single source of truth: it
sets the `.deb` version, the release tag, and the `voyant-sdk` version built
against (versions are lockstep — `voyant-ros vX.Y.Z` is built against
`voyant-sdk vX.Y.Z`).

1. Branch and bump `<version>` in [`package.xml`](package.xml) (e.g. to `0.9.3`).
2. Dry run to validate: **Actions → Build & Release Debian → Run workflow** on
   your branch. A manual run always builds + smoke-tests against
   `voyant-sdk v0.9.3` (the package.xml version) without publishing.
3. Merge to `main`.
4. Tag the matching version and push:

   ```bash
   git tag v0.9.3
   git push origin v0.9.3
   ```

The tag push runs the workflow again and publishes. The tag **must** equal
`v<package.xml version>` — if it doesn't, the `config` job fails fast (bump
`package.xml` to match). If `voyant-sdk v0.9.3` doesn't exist, it likewise fails
fast. Once `build` and `test-install` pass, the release is published on the tag
with auto-generated notes from the commit/PR history.

### Dry run

A manual run never publishes — it always builds + smoke-tests against
`v<package.xml version>`. Use it to validate a branch before tagging:
**Actions → Build & Release Debian → Run workflow**. Publishing happens only on a
`v*` tag push.

> **Note**
> The manual **Run workflow** button only appears once the workflow file is on
> the default branch (`main`). From another branch you can still dispatch it via
> the CLI:
>
> ```bash
> gh workflow run release-debian.yml --ref <branch>
> ```

The manual build steps this automates are documented in [`docs/debian.md`](docs/debian.md).
