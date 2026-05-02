cmake_minimum_required(VERSION 3.10)
project(perseus_lite_explorer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(frontier_explorer src/frontier_explorer.cpp)
ament_target_dependencies(
  frontier_explorer
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS frontier_explorer DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
