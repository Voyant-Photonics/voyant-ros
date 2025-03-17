cmake_minimum_required(VERSION 3.8)
project(voyant-ros)

# ==============================
# Dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)
find_package(foxglove_bridge REQUIRED QUIET)

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

# ==============================
# Package dependencies
# ==============================
ament_target_dependencies(voyant_sensor_node
  rclcpp
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
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
