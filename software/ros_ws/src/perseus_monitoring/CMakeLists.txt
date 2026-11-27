# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_monitoring
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

# INCLUDE DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp geometry_msgs nav_msgs sensor_msgs
                                 perseus_interfaces)
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
# Pure logic, unit-tested without ROS.
add_library(perseus_monitoring_core STATIC src/slip_detector.cpp
                                           src/topic_tracker.cpp)
target_include_directories(perseus_monitoring_core
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(mobility_monitor src/mobility_monitor.cpp)
target_link_libraries(mobility_monitor perseus_monitoring_core)
ament_target_dependencies(mobility_monitor rclcpp geometry_msgs nav_msgs
                          sensor_msgs perseus_interfaces)

add_executable(topic_health_monitor src/topic_health_monitor.cpp)
target_link_libraries(topic_health_monitor perseus_monitoring_core)
ament_target_dependencies(topic_health_monitor rclcpp perseus_interfaces)

# INSTALL
install(TARGETS mobility_monitor topic_health_monitor
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  foreach(test_name slip_detector_test topic_tracker_test)
    ament_add_gtest(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} perseus_monitoring_core)
  endforeach()
endif()

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
