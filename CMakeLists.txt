cmake_minimum_required(VERSION 3.15)
project(voyant-ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Detect ROS 2 distro version automatically
if(DEFINED ENV{ROS_DISTRO})
  message(STATUS "Detected ROS_DISTRO: $ENV{ROS_DISTRO}")

  if("$ENV{ROS_DISTRO}" STREQUAL "rolling")
    add_compile_definitions(ROS2_ROLLING)
  elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    add_compile_definitions(ROS2_JAZZY)
  elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_compile_definitions(ROS2_HUMBLE)
  endif()
endif()

# ==============================
# Dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# ==============================
# Compiler Options
# ==============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# ==============================
# Include & Link Directories
# ==============================
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})

# ==============================
# Executables & Link libraries
# ==============================
add_executable(voyant_sensor_node src/main.cpp src/sensor_driver.cpp)
target_link_libraries(voyant_sensor_node
  voyant-api::voyant-api
  ${PCL_LIBRARIES}
)

# ======= bin2mcap converter =======
add_executable(voyant_bin_to_mcap src/bin_to_mcap.cpp src/voyant_rosbag2_converter.cpp)
target_link_libraries(voyant_bin_to_mcap
  voyant-api::voyant-api
  yaml-cpp
  ${voyant_sensor_node}
  ${PCL_LIBRARIES}
)
set_target_properties(voyant_bin_to_mcap
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ==============================
# Package dependencies
# ==============================
ament_target_dependencies(voyant_sensor_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
ament_target_dependencies(voyant_bin_to_mcap
  rclcpp
  rosbag2_cpp
  sensor_msgs
  pcl_conversions
)

# ==============================
# Install Targets
# ==============================
install(TARGETS
  voyant_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  include/${PROJECT_NAME}/
)
install(
  FILES
  LICENSE
  DESTINATION
  share/${PROJECT_NAME}
)
install(
  DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.rviz"
  PATTERN "*.json")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
