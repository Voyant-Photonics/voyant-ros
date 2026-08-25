# Official ROS2 drivers for Voyant LiDARs

[![CI](https://github.com/Voyant-Photonics/voyant-ros/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/Voyant-Photonics/voyant-ros/actions/workflows/docker-image.yml)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Lyrical-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04%20%7C%2026.04-orange)
![voyant-api](https://img.shields.io/badge/voyant--api-%E2%89%A5%201.0.0-green)
[![License](https://img.shields.io/github/license/Voyant-Photonics/voyant-ros)](LICENSE)
[![Latest release](https://img.shields.io/github/v/release/Voyant-Photonics/voyant-ros)](https://github.com/Voyant-Photonics/voyant-ros/releases)

This ROS2 package provides support for Voyant sensors.
Configure the sensor (client) address using the `config/sensor_params.yaml` file.
It requires [`voyant-api`](https://github.com/Voyant-Photonics/voyant-sdk) **≥ 1.0.0**.
Pre-built releases bundle the version-matched `voyant-api` packages, so you only
install `voyant-api` yourself when building from source — any release ≥ 1.0.0
works there.
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

#### 2. Download and install packages

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

#### 3. [Optional] Install visualization tools

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
#### 2. Install Voyant API

Download `voyant-api_*_amd64.deb` and `voyant-api-dev_*_amd64.deb` from the [latest voyant-sdk release](https://github.com/Voyant-Photonics/voyant-sdk/releases/latest) and install them:

```bash
cd ~/Downloads  # or location of downloaded files
sudo apt update
sudo apt install -y ./voyant-api_*amd64.deb \
                    ./voyant-api-dev_*amd64.deb
```

#### 3. Install ROS dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-rviz2
```

> **Remove any installed `ros-$ROS_DISTRO-voyant-ros` first**
> (`sudo apt remove ros-$ROS_DISTRO-voyant-ros`). Its message definitions take
> precedence over the ones you build, and the driver and converters will silently
> read and write the wrong `VoyantDeviceMetadata`.

#### 4. Clone and build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Voyant-Photonics/voyant-ros.git
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select voyant_ros
```

> **Note**
> You can also install ROS dependencies via rosdep instead of step 3 (run from workspace root after cloning):
>
> ```bash
> rosdep install --from-paths src --ignore-src -r -y
> ```
>
> The pointcloud can also be visualized using RViz — set the `use_rviz` launch argument to `true`.

#### 5. [Optional] Install visualization tools

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

**Upgrading from a pre-1.0.0 driver:** parameters and the published point layout both
changed, and existing param files need editing:

- `point_format` was **removed** — there is now a single point layout, and it is not
  the old `MDL_EXTENDED` under a new name. Against `MDL_STANDARD` it adds
  `calibrated_reflectance` and `frame_index`; against `MDL_EXTENDED` it drops
  `noise_mean_estimate` and `min_ramp_snr`, which the Carbon sensor never populated.
  The packed `point_idx` is replaced by separate `azimuth_idx` and `elevation_idx`
  fields, and `timestamp_nsecs` is now `uint32` rather than `int32`. The live node
  ignores an unrecognized key, so a stale `point_format` entry is only confusing, but
  delete it anyway.
- `valid_only_filter` was **renamed** to `diagnostic_mode`, inverting its sense: set
  it `true` to include invalid returns rather than to exclude them. `voyant_bin_to_mcap`
  reads the YAML directly and exits if the key is missing, so this rename is not
  optional for it. The live node now drops invalid returns by default, where before it
  kept them.
- `mcap_to_bin_params.yaml` gained a **`topic_name`** key naming the bag's point-cloud
  topic. Add it to an existing file. A namespaced capture resolves too — `/point_cloud`
  finds `/front/point_cloud` — but if a bag holds more than one match (two sensors, say)
  the converter refuses to guess: give it the full topic.

`VoyantDeviceMetadata` also changed: the four `*_version_hash` integers became
readable strings. `/device_metadata` now publishes `api_version` and
`interface_contract_version` (the `voyant-api` library this driver links, and the
sensor wire protocol it was built for), `firmware_version` and `hdl_version` read
from the sensor itself, `product_id` and `serial_number` behind the device ID, and
`recording_api_version` — the version that wrote the source recording, set only by
`voyant_bin_to_mcap`. Everything read from the sensor is unset for a frame that
carries no state, such as a recording converted from the pre-1.0.0 format.

> 🚧 **Temporary fix (Carbon sensor)**
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

## Converting between `.vynt` recordings and ROS2 bags

`voyant_bin_to_mcap` converts a `.vynt` recording to an MCAP bag; `voyant_mcap_to_bin`
converts one back. Both ship in the Debian package, so an installed release needs
nothing built. Source the workspace first, exactly as for the driver — see
[Source the ROS2 workspace](#1-source-the-ros2-workspace) — so the
`VoyantDeviceMetadata` message resolves at runtime.

> Recordings made with a pre-1.0.0 `voyant-api` must first be converted to the
> `.vynt` format with the `voyant_recording_migrate` tool from the SDK.

### Config files

Each tool takes the path to a YAML config:

| Config | Used by |
| ------ | ------- |
| `sensor_params.yaml` | `voyant_bin_to_mcap`, and the live driver |
| `mcap_to_bin_params.yaml` | `voyant_mcap_to_bin` |

The packaged copies live in `share/voyant_ros/config/` under the install prefix —
`/opt/ros/$ROS_DISTRO/` for a released package — which is root-owned. Copy them
somewhere writable rather than editing in place:

```bash
cp /opt/ros/$ROS_DISTRO/share/voyant_ros/config/sensor_params.yaml \
   /opt/ros/$ROS_DISTRO/share/voyant_ros/config/mcap_to_bin_params.yaml ~/
```

Paths inside a config are resolved against the directory you run from, not against
the config file — absolute paths are the safe choice.

### `.vynt` → MCAP

Set `bin_input` and `mcap_output` in `sensor_params.yaml`, then:

```bash
ros2 run voyant_ros voyant_bin_to_mcap ~/sensor_params.yaml
```

`mcap_output` is a directory, and rosbag2 refuses to write into one that already
exists — delete it before re-running (see [Cleaning up](#cleaning-up)).

### MCAP → `.vynt`

Set `mcap_input`, `bin_output` and `topic_name` in `mcap_to_bin_params.yaml`.
`bin_output` must end in `.vynt`, and the recorder refuses to overwrite a file that
already exists — delete it before re-running (see [Cleaning up](#cleaning-up)).

```bash
ros2 run voyant_ros voyant_mcap_to_bin ~/mcap_to_bin_params.yaml
```

Only bags carrying the full driver field set and a `/device_metadata` topic convert.
`voyant_bin_to_mcap` writes both automatically; a live `ros2 bag record` capture must
include that topic alongside the points. Bags recorded by a pre-1.0.0 driver are
rejected — their point layout differs.

The rebuilt recording keeps the point data, the frame timeline and the sensor's
identity, but **not the per-frame sensor state**: a bag carries the points, not the
heartbeat behind them, so health, calibration, SDL settings and time-sync figures are
lost. Those frames report their state as absent rather than as zeros, which would
decode into plausible-looking readings (a zeroed temperature becomes -273.15 °C).
Record with `voyant_recorder` if you need the state preserved.

## Cleaning up

colcon has no built-in `clean` verb — there is no equivalent of ROS 1's
`catkin clean` unless you install the optional
[`colcon-clean`](https://github.com/ruffsl/colcon-clean) extension. Removing the
directories directly is the supported way.

**Rebuild this package from scratch** (from the workspace root):

```bash
rm -rf build/voyant_ros install/voyant_ros
colcon build --symlink-install --packages-select voyant_ros
```

Mixing `--symlink-install` with a plain `colcon build` in the same directory fails
with `failed to create symbolic link ... Is a directory`; deleting the two
directories above clears it. For a full workspace reset, remove `build/`, `install/`
and `log/` entirely — all three are regenerated by the next build.

**Remove stale conversion output.** Neither converter overwrites: rosbag2 will not
write into an existing `mcap_output` directory, and the recorder refuses an existing
`.vynt` `bin_output` (it says so by name — `already exists — refusing to overwrite
it`). Delete whichever one you are about to regenerate.

```bash
rm -rf <mcap_output>
rm -f <bin_output>
```

**Remove an older in-repo `build/` directory.** Versions of this README before
v1.0.0 documented building inside `src/voyant-ros/build` with plain `cmake`. That
flow is no longer supported — a bare build tree cannot supply the message typesupport
at runtime — and a leftover directory only invites running its stale binaries.

```bash
rm -rf src/voyant-ros/build src/voyant-ros/install
```

**Remove a system-installed package before building from source**, as covered in
[build from source](#option-2-native--build-from-source) — its message definitions
take precedence over your build.

```bash
sudo apt remove ros-$ROS_DISTRO-voyant-ros
```

## Configuring Foxglove for Pointcloud Visualization

The launch command will start the driver and publish pointcloud data on the `/point_cloud` topic.
It will also open the Foxglove GUI for visualization.

1. Click on `Open connection...` in the Foxglove GUI on the left panel.
2. Connect Foxglove to the default Foxglove websocket server at `ws://localhost:8765`
3. In the top right corner of the Foxglove GUI title bar, click on the `Layout` button and import the configuration file from `config/voyant_foxglove_cfg.json`

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
