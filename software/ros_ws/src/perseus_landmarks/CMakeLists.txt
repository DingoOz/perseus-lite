# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_landmarks
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
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    sensor_msgs
    nav_msgs
    std_srvs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    perseus_interfaces)
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake ament_cmake_python
                               yaml-cpp OpenCV)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
# ROS-free core: landmark map, estimator, OpenCV tag detection. Unit-tested directly.
add_library(perseus_landmarks_core STATIC
            src/landmark_map.cpp src/site_estimator.cpp src/tag_detection.cpp)
set_target_properties(perseus_landmarks_core
                      PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(perseus_landmarks_core
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(perseus_landmarks_core PUBLIC yaml-cpp ${OpenCV_LIBS})
target_include_directories(perseus_landmarks_core PUBLIC ${OpenCV_INCLUDE_DIRS})

add_executable(tag_detector src/tag_detector_node.cpp)
target_link_libraries(tag_detector perseus_landmarks_core)
ament_target_dependencies(tag_detector rclcpp sensor_msgs cv_bridge tf2
                          perseus_interfaces)

add_executable(landmark_localiser src/landmark_localiser_node.cpp)
target_link_libraries(landmark_localiser perseus_landmarks_core)
ament_target_dependencies(
  landmark_localiser
  rclcpp
  nav_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  perseus_interfaces)

# INSTALL
install(TARGETS tag_detector landmark_localiser DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/landmark_navigator.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  foreach(test_name site_estimator_test tag_detection_test)
    ament_add_gtest(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} perseus_landmarks_core)
  endforeach()
  ament_add_pytest_test(landmark_python_test test/test_landmark_python.py)
endif()

# ROS LIBRARY POST-INSTALL
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# ROS FINALISATION
ament_package()
