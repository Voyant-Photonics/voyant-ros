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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  "include;"
)
set(voyant_ros_deps
  rclcpp
  sensor_msgs
  pcl_conversions
)
add_executable(voyant_sensor_node src/main.cpp src/sensor_driver.cpp)

target_link_libraries(voyant_sensor_node
  voyant-api::voyant-api
  ${PCL_LIBRARIES}
)

ament_target_dependencies(voyant_sensor_node
  ${voyant_ros_deps}
)
install(TARGETS
  voyant_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

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
  config
  DESTINATION
  share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_package()
