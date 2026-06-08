cmake_minimum_required(VERSION 3.15)
project(voyant_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Detect ROS 2 distro for distro-specific build logic.
# rosbag2 renamed SerializedBagMessage::time_stamp -> recv_timestamp in Jazzy, so
# flag Humble -- the only supported distro still on the legacy field. Every newer
# distro (Jazzy, Kilted, Lyrical, Rolling, ...) uses recv_timestamp and needs no
# macro. See src/voyant_bin_to_mcap.cpp.
if(DEFINED ENV{ROS_DISTRO})
  message(STATUS "Detected ROS_DISTRO: $ENV{ROS_DISTRO}")
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_compile_definitions(ROS2_HUMBLE)
  endif()
endif()

# ==============================
# Dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ==============================
# Compiler Options
# ==============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

# ==============================
# Dependency linking helper
# ==============================
# ament_target_dependencies() was deprecated in Kilted and REMOVED in Lyrical.
# Keep using it where it still exists (Humble / Jazzy / Kilted); on Lyrical and
# newer, fall back to modern namespaced CMake targets. Almost every ament
# package exports a `<pkg>::<pkg>` target; message packages (e.g. sensor_msgs)
# expose their interface libraries via `${<pkg>_TARGETS}` instead.
function(voyant_target_deps target)
  if(COMMAND ament_target_dependencies)
    ament_target_dependencies(${target} ${ARGN})
  else()
    foreach(pkg ${ARGN})
      if(DEFINED ${pkg}_TARGETS)
        target_link_libraries(${target} ${${pkg}_TARGETS})
      else()
        target_link_libraries(${target} ${pkg}::${pkg})
      endif()
    endforeach()
  endif()
endfunction()

# ==============================
# Include & Link Directories
# ==============================
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})

# ==============================
# Define custom messages
# ==============================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoyantDeviceMetadata.msg"
  DEPENDENCIES std_msgs
)

# ==============================
# Executables & Link libraries
# ==============================
add_executable(voyant_sensor_node src/main.cpp src/sensor_driver.cpp)
voyant_target_deps(voyant_sensor_node
  rclcpp
  sensor_msgs
  pcl_conversions
  ament_index_cpp
)
target_link_libraries(voyant_sensor_node
  voyant-api::voyant-api
  ${PCL_LIBRARIES}
)

# ======= bin2mcap converter =======
add_executable(voyant_bin_to_mcap src/bin_to_mcap.cpp src/voyant_bin_to_mcap.cpp)
voyant_target_deps(voyant_bin_to_mcap
  rclcpp
  sensor_msgs
  pcl_conversions
  ament_index_cpp
  rosbag2_cpp
)
target_link_libraries(voyant_bin_to_mcap
  voyant-api::voyant-api
  ${PCL_LIBRARIES}
)
set_target_properties(voyant_bin_to_mcap
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ======= mcap2bin converter =======
add_executable(voyant_mcap_to_bin src/mcap_to_bin.cpp src/voyant_mcap_to_bin.cpp)
voyant_target_deps(voyant_mcap_to_bin
  rclcpp
  sensor_msgs
  pcl_conversions
  ament_index_cpp
  rosbag2_cpp
)
target_link_libraries(voyant_mcap_to_bin
  voyant-api::voyant-api
  ${PCL_LIBRARIES}
)
set_target_properties(voyant_mcap_to_bin
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ==============================
# Link Custom Message Interfaces
# ==============================
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(voyant_sensor_node "${cpp_typesupport_target}")
target_link_libraries(voyant_bin_to_mcap "${cpp_typesupport_target}")
target_link_libraries(voyant_mcap_to_bin "${cpp_typesupport_target}")

# ==============================
# Install Targets
# ==============================
install(TARGETS
  voyant_sensor_node
  voyant_bin_to_mcap
  voyant_mcap_to_bin
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
