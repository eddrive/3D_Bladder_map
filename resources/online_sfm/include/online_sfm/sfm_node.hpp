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
#include <Eigen/Dense>
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
  // Core processing functions
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  bool getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam);
  void processFrame(const cv::Mat& image, const Eigen::Matrix4d& T);
  
  // Preprocessing functions
  cv::Mat preprocessImage(const cv::Mat& raw_image);
  cv::Mat correctIllumination(const cv::Mat& image);
  cv::Mat adaptiveContrastEnhancement(const cv::Mat& image);
  cv::Mat reduceSpecularReflections(const cv::Mat& gray_image, const cv::Mat& color_image);
  cv::Mat applyUnsharpMask(const cv::Mat& image, double strength, double threshold);
  double detectMotionBlur(const cv::Mat& image);
  void logPreprocessingStats(const cv::Mat& original, const cv::Mat& processed);
  
  // Endoscope-specific functions
  std::vector<cv::KeyPoint> undistortKeypoints(const std::vector<cv::KeyPoint>& keypoints);
  bool isInlierUndistorted(const cv::DMatch& match,
                          const std::vector<cv::KeyPoint>& prev_kpts_undist,
                          const std::vector<cv::KeyPoint>& curr_kpts_undist,
                          const Eigen::Matrix4d& prev_pose,
                          const Eigen::Matrix4d& curr_pose,
                          double threshold);
  
  // Debug functions - NUOVE AGGIUNTE
  void debugCameraPoses(const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2, int frame_num);
  void testProgressiveThresholds(const std::vector<cv::DMatch>& good_matches,
                                const std::vector<cv::KeyPoint>& undistorted_prev_keypoints,
                                const std::vector<cv::KeyPoint>& undistorted_keypoints,
                                const Eigen::Matrix4d& prev_pose,
                                const Eigen::Matrix4d& curr_pose);
  void analyzeRansacFailures(const std::vector<cv::DMatch>& good_matches,
                            const std::vector<cv::KeyPoint>& undistorted_prev_keypoints,
                            const std::vector<cv::KeyPoint>& undistorted_keypoints,
                            const Eigen::Matrix4d& prev_pose,
                            const Eigen::Matrix4d& curr_pose,
                            double threshold);
  
  // Geometric validation
  pcl::PointXYZ triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                 const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2);
  bool isInlier(const cv::DMatch& match, const std::vector<cv::KeyPoint>& prev_kpts,
                const std::vector<cv::KeyPoint>& curr_kpts, const Eigen::Matrix4d& prev_pose,
                const Eigen::Matrix4d& curr_pose, double threshold);
  Eigen::Vector3d projectToCamera(const Eigen::Vector4d& point_3d, 
                                  const Eigen::Matrix4d& camera_pose);
  std::vector<cv::DMatch> applyRANSAC(const std::vector<cv::DMatch>& matches,
                                      const std::vector<cv::KeyPoint>& prev_kpts,
                                      const std::vector<cv::KeyPoint>& curr_kpts,
                                      const Eigen::Matrix4d& prev_pose,
                                      const Eigen::Matrix4d& curr_pose);
  
  // Keyframe triggering
  bool shouldProcessFrame(const Eigen::Matrix4d& current_pose, 
                         const rclcpp::Time& current_time);
  
  // Point management
  void addPointsToMap(const std::vector<pcl::PointXYZ>& new_points,
                      const std::vector<cv::DMatch>& matches,
                      const std::vector<cv::KeyPoint>& current_keypoints);
  double calculatePointQuality(const pcl::PointXYZ& point, 
                               const std::vector<cv::KeyPoint>& observations);
  void filterPointCloud();
  void publishAccumulatedPointCloud();
  
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
  
  // Keyframe tracking
  int keyframe_counter_ = 0;
  rclcpp::Time last_keyframe_time_;
  cv::Mat current_image_;
  Eigen::Matrix4d current_pose_;
  
  // Point cloud management  
  std::vector<PointWithObservations> point_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  
  // Basic parameters
  double min_point_distance_ = 0.05;
  int min_observations_ = 2;
  int max_points_ = 10000;
  double outlier_radius_ = 0.2;
  int outlier_min_neighbors_ = 5;
  double voxel_leaf_size_ = 0.02;
  int publish_every_n_frames_ = 5;
  std::string world_frame_ = "world";
  std::string camera_frame_ = "camera";
  
  // Keyframe triggering (ENDOSCOPE VALUES)
  double min_baseline_meters_ = 0.015;
  double min_rotation_degrees_ = 1.5;
  double max_time_between_keyframes_ = 2.0;
  
  // RANSAC parameters (ENDOSCOPE VALUES)
  int ransac_max_iterations_ = 2000;
  double ransac_threshold_ = 5.0;
  double ransac_confidence_ = 0.99;
  int ransac_min_inliers_ = 12;
  
  // Preprocessing parameters
  bool use_morphological_correction_ = false;
  
  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;
};