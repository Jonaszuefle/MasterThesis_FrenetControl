cmake_minimum_required(VERSION 3.8)
project(real_hardware)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(omni_msgs REQUIRED)

include_directories(
  include
)

add_executable(real_hardware
  src/real_hardware.cpp
)

ament_target_dependencies(real_hardware
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  orocos_kdl
  kdl_parser
  ament_index_cpp
  omni_msgs
)

install(TARGETS
  real_hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource/
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
