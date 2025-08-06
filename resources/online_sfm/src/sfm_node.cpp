#include "online_sfm/sfm_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>

SfMNode::SfMNode() : Node("online_sfm") {
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/endoscope/image_raw", 10,
    std::bind(&SfMNode::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/endoscope/camera_info", 10,
    std::bind(&SfMNode::cameraInfoCallback, this, std::placeholders::_1));

  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sfm/point_cloud", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SfMNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (cam_info_received_) return;
  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat(msg->d).clone();
  cam_info_received_ = true;
}

bool SfMNode::getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam) {
  try {
    auto tf_msg = tf_buffer_->lookupTransform("world", "camera_frame", stamp);
    Eigen::Translation3d t(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);
    Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                         tf_msg.transform.rotation.x,
                         tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    T_world_cam = (t * q).matrix();
    return true;
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed");
    return false;
  }
}

void SfMNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!cam_info_received_) return;

  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

  Eigen::Matrix4d pose;
  if (!getCameraPose(msg->header.stamp, pose)) return;

  processFrame(image, pose);
}

void SfMNode::processFrame(const cv::Mat& image, const Eigen::Matrix4d& T) {
  auto sift = cv::SIFT::create();
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

  if (has_prev_) {
    std::vector<cv::DMatch> matches;
    auto matcher = cv::BFMatcher(cv::NORM_L2);
    matcher.match(prev_descriptors_, descriptors, matches);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& m : matches) {
      cv::Point2f pt1 = prev_keypoints_[m.queryIdx].pt;
      cv::Point2f pt2 = keypoints[m.trainIdx].pt;

      cv::Mat pt1_norm = (cv::Mat_<double>(3,1) << (pt1.x - camera_matrix_.at<double>(0,2)) / camera_matrix_.at<double>(0,0),
                                                   (pt1.y - camera_matrix_.at<double>(1,2)) / camera_matrix_.at<double>(1,1), 1.0);
      cv::Mat pt2_norm = (cv::Mat_<double>(3,1) << (pt2.x - camera_matrix_.at<double>(0,2)) / camera_matrix_.at<double>(0,0),
                                                   (pt2.y - camera_matrix_.at<double>(1,2)) / camera_matrix_.at<double>(1,1), 1.0);

      Eigen::Vector3d ray1(pt1_norm.at<double>(0), pt1_norm.at<double>(1), pt1_norm.at<double>(2));
      Eigen::Vector3d ray2(pt2_norm.at<double>(0), pt2_norm.at<double>(1), pt2_norm.at<double>(2));

      Eigen::Matrix4d T1 = prev_pose_;
      Eigen::Matrix4d T2 = T;

      Eigen::Vector3d p1 = T1.block<3,1>(0,3);
      Eigen::Vector3d p2 = T2.block<3,1>(0,3);
      Eigen::Vector3d d1 = T1.block<3,3>(0,0) * ray1;
      Eigen::Vector3d d2 = T2.block<3,3>(0,0) * ray2;

      Eigen::Vector3d n = d1.cross(d2);
      Eigen::Matrix2d A;
      A << d1.dot(d1), -d1.dot(d2),
           d1.dot(d2), -d2.dot(d2);

      Eigen::Vector2d b = Eigen::Vector2d((p2 - p1).dot(d1), (p2 - p1).dot(d2));
      Eigen::Vector2d lambda = A.inverse() * b;

      Eigen::Vector3d point = 0.5 * (p1 + lambda[0] * d1 + p2 + lambda[1] * d2);
      cloud->points.emplace_back(point.x(), point.y(), point.z());
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = this->now();
    pc_pub_->publish(cloud_msg);
  }

  prev_image_ = image.clone();
  prev_descriptors_ = descriptors.clone();
  prev_keypoints_ = keypoints;
  prev_pose_ = T;
  has_prev_ = true;
}