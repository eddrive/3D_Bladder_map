#include "bladder_rtabmapper/tf_to_odom_publisher.hpp"

TFtoOdomPublisher::TFtoOdomPublisher()
    : Node("tf_to_odom_publisher"),
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_)
{
    
    this->declare_parameter<std::string>("parent_frame", "base_link");
    this->declare_parameter<std::string>("child_frame", "camera_pose");
    this->get_parameter("parent_frame", parent_frame_);
    this->get_parameter("child_frame", child_frame_);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TFtoOdomPublisher::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), 
      "Publishing /odom from TF [%s] -> [%s]", 
      parent_frame_.c_str(), child_frame_.c_str());
}

void TFtoOdomPublisher::timer_callback()
{
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer_.lookupTransform(
            parent_frame_, child_frame_, tf2::TimePointZero
        );
    } catch(const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
          "Could not get transform: %s", ex.what());
        return;
    }
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = parent_frame_;
    msg.child_frame_id = child_frame_;
    msg.pose.pose.position.x = t.transform.translation.x;
    msg.pose.pose.position.y = t.transform.translation.y;
    msg.pose.pose.position.z = t.transform.translation.z;
    msg.pose.pose.orientation = t.transform.rotation;
    // OPTIONAL: Velocità (twist) lasciata a zero
    odom_pub_->publish(msg);
}

// main()
#include <memory>
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFtoOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}