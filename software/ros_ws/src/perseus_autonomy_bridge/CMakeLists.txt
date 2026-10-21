cmake_minimum_required(VERSION 3.10)
project(perseus_autonomy_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perseus_interfaces REQUIRED)

add_executable(nav2_waypoints_bridge src/nav2_waypoints_bridge.cpp)
target_include_directories(nav2_waypoints_bridge
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(nav2_waypoints_bridge rclcpp rclcpp_action nav2_msgs
                          geometry_msgs perseus_interfaces)

install(TARGETS nav2_waypoints_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# TESTING. Fixture names end in "Test" so gtest_discover_tests yields ctest
# names matching CI's "-R 'Test\\.'" filter.
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  add_executable(goal_tracker_test test/goal_tracker_test.cpp)
  target_include_directories(goal_tracker_test
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(goal_tracker_test GTest::gtest_main)
  gtest_discover_tests(goal_tracker_test)
endif()

ament_package()
