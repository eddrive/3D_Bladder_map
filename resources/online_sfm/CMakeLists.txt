cmake_minimum_required(VERSION 3.5)
project(online_sfm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(sfm_node src/sfm_node.cpp)
ament_target_dependencies(sfm_node rclcpp sensor_msgs cv_bridge image_transport tf2 tf2_ros OpenCV)
target_link_libraries(sfm_node ${OpenCV_LIBS} ${PCL_LIBRARIES})

install(TARGETS sfm_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()