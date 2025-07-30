cmake_minimum_required(VERSION 3.5)
project(midas_depth_ros)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(midas_depth_node src/midas_depth_node.cpp)
ament_target_dependencies(midas_depth_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS midas_depth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
