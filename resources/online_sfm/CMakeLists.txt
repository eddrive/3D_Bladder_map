# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(online_sfm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for this project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)

# Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(sfm_node
  src/sfm_node.cpp
)

# Target include directories
target_include_directories(sfm_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Target dependencies
ament_target_dependencies(sfm_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# Link libraries
target_link_libraries(sfm_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS
  sfm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install include headers
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()