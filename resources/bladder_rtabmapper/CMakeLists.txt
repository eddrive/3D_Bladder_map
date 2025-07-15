cmake_minimum_required(VERSION 3.8)
project(bladder_rtabmapper)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(tf_to_odom_publisher src/tf_to_odom_publisher.cpp)
ament_target_dependencies(tf_to_odom_publisher rclcpp tf2_ros nav_msgs geometry_msgs)

install(TARGETS tf_to_odom_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()