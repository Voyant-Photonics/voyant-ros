
# [WIP] voyant-ros debian

## Generating a debian

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

Then generate:

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

### Test package in a clean docker container

Start from a directory that has:

```bash
$ ls
ros-humble-voyant-ros_0.1.0-0jammy_amd64.deb  voyant-api_0.2.1-1_amd64.deb  voyant-api-dev_0.2.1-1_amd64.deb
```

Then run a clean ROS humble docker container:

```bash
docker run -it --rm -v $(pwd):/debs osrf/ros:humble-desktop
```

Install:

```bash
apt update
apt install -y /debs/voyant-api*.de
apt install -y /debs/ros-humble-voyant-ros*.deb
```
