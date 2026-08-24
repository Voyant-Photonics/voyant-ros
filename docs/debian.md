# [WIP] voyant_ros debian

## Setup

Install deps:

```bash
sudo apt update
sudo apt install build-essential debhelper python3-bloom
```

Trick rosdep:

```bash
# Create a local rosdep yaml file
mkdir -p ~/rosdep
cat > ~/rosdep/local.yaml << EOF
voyant-api:
  ubuntu:
    jammy: [voyant-api-dev]      # humble (Ubuntu 22.04)
    noble: [voyant-api-dev]      # jazzy / kilted (Ubuntu 24.04)
    resolute: [voyant-api-dev]   # lyrical (Ubuntu 26.04)
EOF

# Add it to rosdep sources
echo "yaml file:///$HOME/rosdep/local.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-local.list

# Update rosdep
rosdep update
```

IFF you fail this:

```bash
# This command finds the problematic line (if it exists) and comments it out.
# It's safe to run even if the file or line doesn't exist.
sudo sed -i '/debian.yaml/s/^/# /' /etc/ros/rosdep/sources.list.d/10-debian.list
```

And try `rosdep update` again.

## Generate a release

From `voyant-ros/` generate a debian with (swap `--os-version` / `--ros-distro`
for your target — `jammy`+`humble`, `noble`+`jazzy`, `noble`+`kilted`, or
`resolute`+`lyrical`):

```bash
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble

fakeroot debian/rules binary
```

### Install the package

```bash
cd ..
sudo dpkg -i ros-humble-voyant-ros*.deb
```

### Test the package

```bash
source /opt/ros/humble/setup.bash
ros2 run voyant_ros voyant_sensor_node
```

Visualize in another terminal

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Test package in a clean docker container

Start from a directory that has the latest releases, e.g.,:

```bash
$ ls debs/
ros-humble-voyant-ros_1.0.0-0jammy_amd64.deb  voyant-api_1.0.0-1_amd64.deb  voyant-api-dev_1.0.0-1_amd64.deb
```

Then run a clean ROS humble docker container:

```bash
docker run -it --rm --name voyant_ros_container --network host -v $(pwd):/workspace --workdir /workspace osrf/ros:humble-desktop
```

Install the debian's:

```bash
apt update
apt install -y /workspace/debs/voyant-api*.deb
apt install -y /workspace/debs/ros-humble-voyant-ros*.deb
apt install -y ros-humble-foxglove-* # for visualization
```

Run the node:

```bash
ros2 run voyant_ros voyant_sensor_node
```

> NOTE: To smoke test against the simulator (no real sensor), first override
> the bind interface in the installed config to loopback:
>
> ```bash
> sed -i "s|interface_address:.*|interface_address: '127.0.0.1'|" \
>   /opt/ros/humble/share/voyant_ros/config/sensor_params.yaml
> ```
>
> Then run the node with that params file:
>
> ```bash
> ros2 run voyant_ros voyant_sensor_node --ros-args \
>   --params-file /opt/ros/humble/share/voyant_ros/config/sensor_params.yaml
> ```

Run the Carbon simulator in terminal 2:

> This can be run outside the docker if you have `voyant-api` installed on host

```bash
docker exec -it voyant_ros_container bash
voyant_simulator --bind-addr 127.0.0.1:0 --group-addr 239.255.48.84:5678
```

> NOTE: The simulator was named `voyant_carbon_simulator` in pre-1.0.0
> releases. Run with `--help` to confirm available flags for your installed
> `voyant-api` version.

Run the ROS2 foxglove bridge in terminal 3:

```bash
docker exec -it voyant_ros_container bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Clean up the debian build

```bash
cd <path/to>/voyant-ros/
```

```bash
# Remove the build artifacts
fakeroot debian/rules clean
rm -rf debian

# Remove the generated package files
rm -f ../ros-humble-voyant-ros*
```
