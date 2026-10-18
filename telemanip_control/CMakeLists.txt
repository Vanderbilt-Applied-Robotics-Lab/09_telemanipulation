cmake_minimum_required(VERSION 3.8)
project(telemanip_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

## INCLUDE(S)
include_directories(include)

# executables
add_executable(telemanip_control_node src/telemanip_control_node.cpp)

# dependencies
ament_target_dependencies(telemanip_control_node rclcpp sensor_msgs geometry_msgs moveit_ros_planning_interface tf2 tf2_ros)

install(TARGETS telemanip_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
