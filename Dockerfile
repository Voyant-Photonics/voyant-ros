ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-core AS build-env

# Set shell options for better error handling
SHELL ["/bin/bash", "-c"]

ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARG VIZ_BRIDGE="false"
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION_ARG=${RMW_IMPLEMENTATION} \
    VIZ_BRIDGE_ARG=${VIZ_BRIDGE} \
    BUILD_HOME=/var/lib/build

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-ament-index-cpp \
    libpcl-dev \
    build-essential \
    apt-utils \
    cmake \
    curl \
    g++ \
    pkg-config \
    jq \
    libwayland-dev \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR ${BUILD_HOME}

RUN if [ "$RMW_IMPLEMENTATION_ARG" = "rmw_cyclonedds_cpp" ]; then \
    apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Install Cap'n Proto from source used by Voyant API
WORKDIR /tmp
RUN curl -O https://capnproto.org/capnproto-c++-1.1.0.tar.gz && \
    tar zxf capnproto-c++-1.1.0.tar.gz && \
    cd capnproto-c++-1.1.0 && \
    ./configure && \
    make -j6 && \
    make install && \
    ldconfig && \
    cd .. && \
    rm -rf capnproto-c++-1.1.0*

# Set working directory
WORKDIR /workspace

# Download and install the latest release packages from GitHub
RUN ARCH=$(dpkg --print-architecture) && \
    TARBALL_URL=$(curl -s https://api.github.com/repos/Voyant-Photonics/voyant-sdk/releases/latest | \
        jq -r '.assets[] | select(.name | endswith(".tar.gz")) | .browser_download_url' | head -n 1) && \
    if [ -z "$TARBALL_URL" ]; then echo "No .tar.gz asset found in latest release" >&2; exit 1; fi && \
    wget -O /tmp/voyant-api.tar.gz "$TARBALL_URL" && \
    tar -xzf /tmp/voyant-api.tar.gz -C /tmp && \
    API_DEB=$(find /tmp -name "voyant-api_*${ARCH}.deb" | head -n 1) && \
    DEV_DEB=$(find /tmp -name "voyant-api-dev_*${ARCH}.deb" | head -n 1) && \
    if [ -z "$API_DEB" ] || [ -z "$DEV_DEB" ]; then echo "Required .deb packages for ARCH=$ARCH not found" >&2; exit 1; fi && \
    apt-get install -y "$API_DEB" "$DEV_DEB" && \
    rm -rf /tmp/voyant-api* /var/lib/apt/lists/*

# Create a workspace directory
WORKDIR /ros2_ws/src

# Copy contents of current dir into the workspace
COPY . ./voyant-ros

# Go back to workspace root
WORKDIR /ros2_ws

# Build the ROS2 package
RUN source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# Install foxglove dependencies if VIZ_BRIDGE_ARG is true
RUN if [ "$VIZ_BRIDGE_ARG" = "true" ]; then \
    apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-foxglove-bridge && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Set up entrypoint
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
# Entry point
ENTRYPOINT source /ros2_ws/install/setup.bash && \
    echo "VIZ_BRIDGE_ARG value: $VIZ_BRIDGE_ARG" && \
    if [ "$VIZ_BRIDGE_ARG" = "true" ]; then \
    echo "Starting with Foxglove bridge enabled" && \
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml & \
    ros2 launch voyant_ros sensor_driver_minimal_launch.py; \
    else \
    echo "Starting with minimal sensor driver" && \
    ros2 launch voyant_ros sensor_driver_minimal_launch.py; \
    fi
