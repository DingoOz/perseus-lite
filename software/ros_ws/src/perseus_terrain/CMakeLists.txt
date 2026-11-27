# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_terrain
  VERSION 0.1.0
  LANGUAGES CXX)
# credit https://www.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
# we always want debug info for stack tracing, so switch to RelWithDebInfo from
# Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()
# Add -Werror flag for release builds
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# INCLUDE DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    sensor_msgs
    nav_msgs
    nav2_msgs
    nav2_costmap_2d
    pluginlib
    geometry_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs)
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake Eigen3)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
# Core algorithms: no ROS, unit-tested directly.
add_library(
  perseus_terrain_core SHARED
  src/grid_geometry.cpp src/terrain_classifier.cpp src/log_odds_map.cpp
  src/footprint.cpp src/depth_projection.cpp src/cost_mapping.cpp)
target_include_directories(
  perseus_terrain_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# The terrain_mapper node.
add_executable(terrain_mapper src/terrain_mapper.cpp
                              src/terrain_mapper_main.cpp)
target_link_libraries(terrain_mapper perseus_terrain_core Eigen3::Eigen)
ament_target_dependencies(
  terrain_mapper
  rclcpp
  sensor_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs)

# The Nav2 costmap layer plugin.
add_library(perseus_terrain_layer SHARED src/terrain_grid_layer.cpp)
target_link_libraries(perseus_terrain_layer perseus_terrain_core)
ament_target_dependencies(
  perseus_terrain_layer
  rclcpp
  nav_msgs
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d
                                         terrain_layer_plugin.xml)

# INSTALL
install(
  TARGETS perseus_terrain_core perseus_terrain_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS terrain_mapper DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  foreach(test_name grid_geometry_test terrain_classifier_test
                    log_odds_map_test depth_projection_test cost_mapping_test)
    ament_add_gtest(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} perseus_terrain_core)
  endforeach()
endif()

# ROS LIBRARY POST-INSTALL
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
