cmake_minimum_required(VERSION 3.8)
project(voyant-ros)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)

# Compiling options
add_compile_options(-Wall -Wextra)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

include_directories(
  "include;"
)
set(voyant_ros_deps
  rclcpp
  sensor_msgs
  pcl_conversions
)
add_executable(voyant_sensor_node src/sensor_node.cpp)
target_link_libraries(voyant_sensor_node
  PRIVATE voyant-api::voyant-api
  ${voyant_ros_deps}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
