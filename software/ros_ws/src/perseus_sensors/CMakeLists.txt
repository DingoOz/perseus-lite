# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_sensors
  VERSION 0.0.1
  LANGUAGES CXX)
cmake_policy(SET CMP0167 NEW)
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

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# INCLUDE DEPENDENCIES

# public dependencies (which should propagate)
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp rclcpp_components sensor_msgs
                                 geometry_msgs nav_msgs)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# fd_wrapper (software/shared) isn't packaged for RoboStack: build it in-tree
find_package(fd_wrapper QUIET)
if(NOT fd_wrapper_FOUND)
  message(STATUS "fd_wrapper not found, adding subdirectory...")
  add_subdirectory(../../../shared/fd-wrapper fd-wrapper)
endif()

# COMPILE (Executables)

# I2C IMU Driver
# Explicit source list - build system tracks dependencies correctly
add_executable(
  i2c_imu_node
  i2c_imu_driver/src/main.cpp i2c_imu_driver/src/i2c_device.cpp
  i2c_imu_driver/src/i2c_imu_node.cpp i2c_imu_driver/src/imu_device_config.cpp)

target_include_directories(
  i2c_imu_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/i2c_imu_driver/include)

ament_target_dependencies(i2c_imu_node PUBLIC rclcpp sensor_msgs)
target_link_libraries(i2c_imu_node PUBLIC fd_wrapper)

# Topic Remapper
# Dynamic topic remapper that detects message type and republishes at lower frequency
add_executable(topic_remapper src/topic_remapper/main.cpp
                              src/topic_remapper/topic_remapper.cpp)

target_include_directories(topic_remapper
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(topic_remapper PUBLIC rclcpp)

# Install ROS Executables
install(TARGETS i2c_imu_node topic_remapper DESTINATION lib/${PROJECT_NAME})

add_library(bias_estimator_component SHARED
            src/bias_remover/bias_estimator_component.cpp)
ament_target_dependencies(bias_estimator_component rclcpp rclcpp_components
                          sensor_msgs geometry_msgs nav_msgs)
target_include_directories(bias_estimator_component
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
rclcpp_components_register_nodes(bias_estimator_component
                                 "imu_processors::BiasEstimator")

add_library(bias_remover_component SHARED
            src/bias_remover/bias_remover_component.cpp)
ament_target_dependencies(bias_remover_component rclcpp rclcpp_components
                          sensor_msgs geometry_msgs)
rclcpp_components_register_nodes(bias_remover_component
                                 "imu_processors::BiasRemover")

install(
  TARGETS bias_estimator_component bias_remover_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch
install(DIRECTORY launch config i2c_imu_driver/launch i2c_imu_driver/config
        DESTINATION share/${PROJECT_NAME})

# TESTING
if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  enable_testing()

  add_executable(i2c_imu_helpers_test tests/i2c_imu_helpers_test.cpp
                                      i2c_imu_driver/src/imu_device_config.cpp)
  target_include_directories(
    i2c_imu_helpers_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/i2c_imu_driver/include)
  target_link_libraries(i2c_imu_helpers_test GTest::gtest_main)

  add_executable(stationary_gate_test tests/stationary_gate_test.cpp)
  target_include_directories(stationary_gate_test
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(stationary_gate_test GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(i2c_imu_helpers_test)
  gtest_discover_tests(stationary_gate_test)
endif()

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
