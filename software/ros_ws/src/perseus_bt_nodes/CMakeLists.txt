cmake_minimum_required(VERSION 3.10)
project(perseus_bt_nodes)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/register_nodes.cpp)

find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)

ament_target_dependencies(${PROJECT_NAME} behaviortree_cpp geometry_msgs
                          builtin_interfaces rclcpp)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

# TESTING. GoogleTest fixtures are named *Test so gtest_discover_tests yields
# ctest names matching CI's "-R 'Test\\.'" filter.
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(nav2_behavior_tree REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  include(GoogleTest)

  # Nav2's BT plugin libraries live next to nav2_behavior_tree's own lib.
  get_filename_component(NAV2_BT_LIB_DIR
                         "${nav2_behavior_tree_DIR}/../../../lib" ABSOLUTE)
  # The trees under test belong to the autonomy package (same source tree).
  get_filename_component(
    AUTONOMY_BT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autonomy/behavior_trees"
    ABSOLUTE)

  add_executable(goal_nodes_test test/goal_nodes_test.cpp)
  add_executable(waypoint_tree_test test/waypoint_tree_test.cpp)
  foreach(test_target goal_nodes_test waypoint_tree_test)
    add_dependencies(${test_target} ${PROJECT_NAME})
    target_compile_definitions(
      ${test_target}
      PRIVATE PERSEUS_BT_NODES_LIB="$<TARGET_FILE:${PROJECT_NAME}>"
              NAV2_BT_LIB_DIR="${NAV2_BT_LIB_DIR}"
              AUTONOMY_BT_DIR="${AUTONOMY_BT_DIR}")
    ament_target_dependencies(${test_target} behaviortree_cpp geometry_msgs)
    target_link_libraries(${test_target} GTest::gtest_main)
    gtest_discover_tests(${test_target})
  endforeach()
  target_include_directories(waypoint_tree_test
                             PRIVATE ${nav2_behavior_tree_INCLUDE_DIRS})
  target_link_libraries(waypoint_tree_test tinyxml2::tinyxml2)
endif()

ament_package()
