#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

class SfMNode : public rclcpp::Node {
public:
  SfMNode();

private:
  // Callbacks
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  
  // Core functions
  void processFrame(const cv::Mat& image, const Eigen::Matrix4d& pose);
  bool getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam);
  
  // Utility - SIGNATURE CORRETTE PER COLORE
  std::vector<cv::KeyPoint> undistortKeypoints(const std::vector<cv::KeyPoint>& keypoints);
  pcl::PointXYZRGB triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                    const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2,
                                    const cv::Mat& image1, const cv::Mat& image2);
  void colorByDepth(pcl::PointXYZRGB& point, double depth);
  void publishPointCloud();
  
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  
  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Camera
  cv::Mat camera_matrix_, dist_coeffs_;
  double fx_, fy_, cx_, cy_;
  bool is_fisheye_ = false;
  bool cam_info_received_ = false;
  
  // SfM - TUTTO CORRETTO PER RGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
  std::vector<cv::KeyPoint> prev_keypoints_;
  cv::Mat prev_descriptors_;
  cv::Mat prev_image_undistorted_; 
  Eigen::Matrix4d prev_pose_;
  bool has_prev_frame_ = false;
  bool has_prev_pose_ = false;
  int keyframe_count_ = 0;
  
  // Parameters
  double min_baseline_meters_;
  double min_rotation_degrees_;
  double ransac_threshold_;
};