# [WIP] voyant-ros debian

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
    jammy: [voyant-api-dev]
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

From `voyant-ros/` generate a debian with:

```bash
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble

fakeroot debian/rules binary
```

See fakeroot failing from not finding cap'n proto as an apt package:

```bash
dpkg-shlibdeps: error: no dependency information found for /usr/local/lib/libcapnp-1.1.0.so (used by debian/ros-humble-voyant-ros/opt/ros/humble/lib/voyant-ros/voyant_sensor_node)
Hint: check if the library actually comes from a package.
dh_shlibdeps: error: dpkg-shlibdeps -Tdebian/ros-humble-voyant-ros.substvars -l/home/kyle/ros2_ws/src/voyant-ros/debian/ros-humble-voyant-ros//opt/ros/humble/lib/ -l/home/kyle/ros2_ws/src/voyant-ros/debian/ros-humble-voyant-ros//opt/ros/humble/opt/voyant-ros/lib/ debian/ros-humble-voyant-ros/opt/ros/humble/lib/voyant-ros/voyant_sensor_node returned exit code 2
dh_shlibdeps: error: Aborting due to earlier error
make[1]: *** [debian/rules:59: override_dh_shlibdeps] Error 2
make[1]: Leaving directory '/home/kyle/ros2_ws/src/voyant-ros'
make: *** [debian/rules:27: binary] Error 2
```

Fix fakeroot failing:

```bash
sed -i '/^override_dh_shlibdeps:/,/^$/s/dh_shlibdeps/dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info/' debian/rules
```

And run it again:

```bash
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
ros2 run voyant-ros voyant_sensor_node
```

Visualize in another terminal

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Test package in a clean docker container

Start from a directory that has:

```bash
$ ls debs/
ros-humble-voyant-ros_0.2.1-0jammy_amd64.deb  voyant-api_0.2.1-1_amd64.deb  voyant-api-dev_0.2.1-1_amd64.deb
```

Then run a clean ROS humble docker container:

```bash
docker run -it --rm --name voyant_ros_container --network host -v $(pwd):/workspace --workdir /workspace osrf/ros:humble-desktop
```

Install the cap'n proto dependency from source:

```bash
curl -O https://capnproto.org/capnproto-c++-1.1.0.tar.gz
tar zxf capnproto-c++-1.1.0.tar.gz
cd capnproto-c++-1.1.0
./configure
make -j6 check
sudo make install
```

> NOTE: This is currently required for `ros-humble-voyant-ros`
>
> To get around this we either need to:
>
> 1. Statically compile Cap'n Proto into `ros-humble-voyant-ros`
> 2. Compile Cap'n proto into a debian or host as an apt package
> 3. Remove the Cap'n Proto dependency from the API

Install the debians:

```bash
apt update
apt install -y /workspace/debs/voyant-api*.deb
apt install -y /workspace/debs/ros-humble-voyant-ros*.deb
apt install -y ros-humble-foxglove-* # for visualization
```

Run the node:

```bash
ros2 run voyant-ros voyant_sensor_node
```

Run the mock points stream in terminal 2:

> This can be run outside the docker if you have `voyant-api` installed on host

```bash
docker exec -it voyant_ros_container bash
voyant_points_mock_stream --bind-addr 127.0.0.1:0 --group-addr 224.0.0.0:4444
```

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
