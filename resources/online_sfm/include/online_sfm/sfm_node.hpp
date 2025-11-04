#ifndef SFM_NODE_HPP_
#define SFM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <memory>
#include <vector>

class SfMNode : public rclcpp::Node
{
public:
  SfMNode();
  ~SfMNode() = default;

private:
  // === CALLBACK FUNCTIONS ===
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  
  // === CORE SFM FUNCTIONS ===
  void processFrame(const cv::Mat& image, const rclcpp::Time& timestamp);
  bool getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& pose);
  bool shouldProcessFrame(const Eigen::Matrix4d& current_pose);
  
  // === IMAGE PROCESSING ===
  cv::Mat preprocessImage(const cv::Mat& input);
  cv::Mat undistortImage(const cv::Mat& input);
  
  // === FEATURE DETECTION AND MATCHING ===
  void detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
  std::vector<cv::DMatch> matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2);
  std::vector<cv::DMatch> filterMatches(const std::vector<cv::DMatch>& matches,
                                        const std::vector<cv::KeyPoint>& kp1,
                                        const std::vector<cv::KeyPoint>& kp2);
  
  // === TRIANGULATION ===
  void triangulateAndAddPoints(const std::vector<cv::DMatch>& matches,
                               const std::vector<cv::KeyPoint>& kp1,
                               const std::vector<cv::KeyPoint>& kp2,
                               const Eigen::Matrix4d& pose1,
                               const Eigen::Matrix4d& pose2,
                               const cv::Mat& image1,
                               const cv::Mat& image2);
  
  pcl::PointXYZRGB triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                  const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2,
                                  const cv::Mat& image2);
  
  bool validateTriangulatedPoint(const pcl::PointXYZRGB& point,
                                const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2);
  
  // === POINT CLOUD MANAGEMENT ===
  void addPointToCloud(const pcl::PointXYZRGB& point);
  void filterPointCloud();
  void publishPointCloud();
  void publishTrajectory();
  
  // === UTILITY FUNCTIONS ===
  void extractColor(pcl::PointXYZRGB& point, const cv::KeyPoint& kp, const cv::Mat& image);
  void logFrameStatistics(int frame_count, int features, int matches, int triangulated);
  void logDiagnostics(const cv::Mat& raw_image, const cv::Mat& processed_image,
                     const std::vector<cv::KeyPoint>& keypoints);
  
  // === PARAMETER MANAGEMENT ===
  void declareParameters();
  void loadParameters();
  
  // === ROS2 PUBLISHERS AND SUBSCRIBERS ===
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  
  // === TF ===
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // === CAMERA PARAMETERS ===
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_info_received_;
  bool is_fisheye_;
  double fx_, fy_, cx_, cy_;
  
  // === FEATURE DETECTION ===
  cv::Ptr<cv::SIFT> sift_detector_;
  cv::Ptr<cv::CLAHE> clahe_;
  
  // === FRAME STATE ===
  bool has_previous_frame_;
  cv::Mat previous_image_;
  std::vector<cv::KeyPoint> previous_keypoints_;
  cv::Mat previous_descriptors_;
  Eigen::Matrix4d previous_pose_;
  rclcpp::Time previous_timestamp_;
  
  // === POINT CLOUD ===
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter_;
  
  // === TRAJECTORY ===
  nav_msgs::msg::Path trajectory_;
  
  // === STATISTICS ===
  int frame_count_;
  int total_features_detected_;
  int total_matches_found_;
  int total_points_triangulated_;
  
  // === CONFIGURABLE PARAMETERS ===
  
  // Motion thresholds
  double min_baseline_meters_;
  double min_rotation_degrees_;
  
  // SIFT parameters
  int sift_max_features_;
  int sift_octave_layers_;
  double sift_contrast_threshold_;
  double sift_edge_threshold_;
  double sift_sigma_;
  
  // Image processing
  bool enable_clahe_;
  double clahe_clip_limit_;
  int clahe_tile_size_;
  bool enable_sharpening_;
  double sharpening_strength_;
  
  // Matching parameters
  double lowe_ratio_threshold_;
  int min_matches_for_triangulation_;
  bool enable_cross_check_;
  
  // Triangulation validation
  double min_triangulation_angle_deg_;
  double max_triangulation_angle_deg_;
  double max_reprojection_error_px_;
  double min_depth_meters_;
  double max_depth_meters_;
  double max_triangulation_error_meters_;
  int image_margin_px_;
  
  // Point cloud filtering
  bool enable_voxel_filtering_;
  double voxel_leaf_size_;
  bool enable_outlier_filtering_;
  int outlier_mean_k_;
  double outlier_std_threshold_;
  
  // Publishing
  std::string world_frame_;
  std::string camera_frame_;
  int publish_every_n_frames_;
  bool publish_trajectory_;
  
  // Diagnostics
  bool enable_detailed_logging_;
  bool enable_image_diagnostics_;
  int log_every_n_frames_;
};

#endif  // SFM_NODE_HPP_