cmake_minimum_required(VERSION 3.16)
project(perseus_lite_screen)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

add_executable(perseus_lite_screen src/main.cpp src/map_screen_node.cpp
                                   src/map_view_widget.cpp)

ament_target_dependencies(
  perseus_lite_screen
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

target_link_libraries(perseus_lite_screen Qt6::Core Qt6::Gui Qt6::Widgets)

install(TARGETS perseus_lite_screen DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
