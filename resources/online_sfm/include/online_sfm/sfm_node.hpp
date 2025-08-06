#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_set>
#include <random>

struct PointWithObservations {
  pcl::PointXYZ point;
  std::vector<int> frame_ids;
  std::vector<cv::KeyPoint> observations;
  int observation_count;
  double quality_score;
};

class SfMNode : public rclcpp::Node {
public:
  SfMNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  bool getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam);
  void processFrame(const cv::Mat& image, const Eigen::Matrix4d& T);
  
  // Point management functions
  void addPointsToMap(const std::vector<pcl::PointXYZ>& new_points, 
                      const std::vector<cv::DMatch>& matches,
                      const std::vector<cv::KeyPoint>& current_keypoints);
  void publishAccumulatedPointCloud();
  void filterPointCloud();
  double calculatePointQuality(const pcl::PointXYZ& point, const std::vector<cv::KeyPoint>& observations);
  
  // RANSAC functions
  std::vector<cv::DMatch> applyRANSAC(const std::vector<cv::DMatch>& matches,
                                       const std::vector<cv::KeyPoint>& prev_kpts,
                                       const std::vector<cv::KeyPoint>& curr_kpts,
                                       const Eigen::Matrix4d& prev_pose,
                                       const Eigen::Matrix4d& curr_pose);
  bool isInlier(const cv::DMatch& match,
                const std::vector<cv::KeyPoint>& prev_kpts,
                const std::vector<cv::KeyPoint>& curr_kpts,
                const Eigen::Matrix4d& prev_pose,
                const Eigen::Matrix4d& curr_pose,
                double threshold = 2.0);
  pcl::PointXYZ triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                 const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2);

  // ROS components
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera parameters
  cv::Mat camera_matrix_, dist_coeffs_;
  bool cam_info_received_ = false;
  double fx_, fy_, cx_, cy_;

  // Frame tracking
  std::vector<cv::KeyPoint> prev_keypoints_;
  cv::Mat prev_descriptors_;
  cv::Mat prev_image_;
  Eigen::Matrix4d prev_pose_;
  bool has_prev_ = false;
  int frame_counter_ = 0;

  // Point cloud management
  std::vector<PointWithObservations> point_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  
  // Parameters
  double min_point_distance_ = 0.05;  // Minimum distance between points
  int min_observations_ = 2;          // Minimum observations to keep a point
  int max_points_ = 10000;           // Maximum points to maintain
  double outlier_radius_ = 0.2;      // Radius for outlier removal
  int outlier_min_neighbors_ = 5;    // Min neighbors in radius
  double voxel_leaf_size_ = 0.02;    // Voxel grid leaf size
  int publish_every_n_frames_ = 5;   // Publish accumulated cloud every N frames
  std::string world_frame_ = "world";
  std::string camera_frame_ = "camera";
  
  // RANSAC parameters
  int ransac_max_iterations_ = 1000;
  double ransac_threshold_ = 3.0;    // Pixel threshold
  double ransac_confidence_ = 0.99;
  int ransac_min_inliers_ = 20;

  // Random number generator for RANSAC
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;
};