#include "midas_depth_ros/midas_depth_node.hpp"

namespace midas_depth_ros
{

MidasDepthNode::MidasDepthNode()
: Node("midas_depth_node")
{
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "endoscope/image_raw", 10,
    std::bind(&MidasDepthNode::image_callback, this, std::placeholders::_1));

  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("endoscope/depth_image", 10);
}

void MidasDepthNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat rgb_image = cv_ptr->image;

  // Qui va il modello MiDaS o la tua logica di stima profondità
  // Per ora, creiamo un'immagine finta di profondità
  cv::Mat depth = cv::Mat::ones(rgb_image.size(), CV_32FC1) * 1.0f;

  auto depth_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth).toImageMsg();
  depth_pub_->publish(*depth_msg);
}

}  // namespace midas_depth_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<midas_depth_ros::MidasDepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
