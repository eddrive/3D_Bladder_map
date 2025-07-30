#ifndef MIDAS_DEPTH_ROS__MIDAS_DEPTH_NODE_HPP_
#define MIDAS_DEPTH_ROS__MIDAS_DEPTH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace midas_depth_ros
{

class MidasDepthNode : public rclcpp::Node
{
public:
  MidasDepthNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

}  // namespace midas_depth_ros

#endif  // MIDAS_DEPTH_ROS__MIDAS_DEPTH_NODE_HPP_
