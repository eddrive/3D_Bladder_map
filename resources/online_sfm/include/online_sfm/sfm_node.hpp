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
#include <map>
#include <deque>

struct PointWithObservations {
  pcl::PointXYZ point;
  std::vector<int> frame_ids;
  std::vector<int> keyframe_ids;           // NEW: Track which keyframes observe this point
  std::vector<cv::KeyPoint> observations;
  int observation_count;
  double quality_score;
  rclcpp::Time first_seen;                 // NEW: When first observed
  rclcpp::Time last_seen;                  // NEW: When last observed
};

struct Keyframe {
  int id;
  rclcpp::Time timestamp;
  cv::Mat image;                           // Store original image for debugging
  cv::Mat processed_image;                 // Store preprocessed version
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  Eigen::Matrix4d pose;
  double quality_score;                    // How good is this keyframe
  std::vector<int> point_ids;              // Points observed in this keyframe
  
  // Quality metrics
  int num_matches_with_prev = 0;           // Matches with previous keyframes
  double motion_blur_score = 0.0;          // Blur assessment
  double feature_density = 0.0;            // Features per area
};

struct KeyframeMatch {
  int keyframe1_id;
  int keyframe2_id; 
  std::vector<cv::DMatch> matches;
  std::vector<cv::DMatch> inlier_matches;  // After RANSAC
  double geometric_score;                   // Quality of geometric fit
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
  
  // Keyframe management
  bool shouldCreateKeyframe(const Eigen::Matrix4d& current_pose, const cv::Mat& image,
                           const std::vector<cv::KeyPoint>& keypoints);
  void createNewKeyframe(const cv::Mat& image, const cv::Mat& processed_image,
                        const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors,
                        const Eigen::Matrix4d& pose);
  void pruneKeyframes();
  double calculateKeyframeQuality(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints,
                                 int num_matches, double motion_blur_score);
  
  // Multi-keyframe matching
  std::vector<KeyframeMatch> matchWithKeyframes(const std::vector<cv::KeyPoint>& current_keypoints,
                                               const cv::Mat& current_descriptors,
                                               const Eigen::Matrix4d& current_pose);
  bool shouldMatchWithKeyframe(const Keyframe& kf, const Eigen::Matrix4d& current_pose);
  KeyframeMatch matchTwoKeyframes(const Keyframe& kf1, const std::vector<cv::KeyPoint>& kf2_keypoints,
                                 const cv::Mat& kf2_descriptors, const Eigen::Matrix4d& kf2_pose);
  
  // Multi-view triangulation
  void triangulateMultiView(const std::vector<KeyframeMatch>& keyframe_matches,
                           const std::vector<cv::KeyPoint>& current_keypoints,
                           const Eigen::Matrix4d& current_pose);
  std::vector<pcl::PointXYZ> triangulateFromMultipleViews(
    const std::vector<std::pair<int, cv::KeyPoint>>& observations,  // keyframe_id, keypoint
    const std::vector<Eigen::Matrix4d>& poses);
  
  // Geometric validation
  pcl::PointXYZ triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                 const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2);
  bool isInlier(const cv::DMatch& match, const std::vector<cv::KeyPoint>& prev_kpts,
                const std::vector<cv::KeyPoint>& curr_kpts, const Eigen::Matrix4d& prev_pose,
                const Eigen::Matrix4d& curr_pose, double threshold);
  std::vector<cv::DMatch> applyRANSAC(const std::vector<cv::DMatch>& matches,
                                      const std::vector<cv::KeyPoint>& prev_kpts,
                                      const std::vector<cv::KeyPoint>& curr_kpts,
                                      const Eigen::Matrix4d& prev_pose,
                                      const Eigen::Matrix4d& curr_pose);
  
  // Point management (enhanced)
  void addPointsToMap(const std::vector<pcl::PointXYZ>& new_points,
                      const std::vector<cv::DMatch>& matches,
                      const std::vector<cv::KeyPoint>& current_keypoints,
                      int current_keyframe_id = -1);  // NEW: Track keyframe association
  void updatePointObservations(int point_id, int keyframe_id, const cv::KeyPoint& keypoint);
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
  int frame_counter_ = 0;
  
  // Keyframe storage
  std::deque<Keyframe> keyframes_;          // Sliding window of keyframes
  int next_keyframe_id_ = 0;                // Unique keyframe IDs
  int current_keyframe_id_ = -1;            // ID of most recent keyframe
  
  // Point cloud management  
  std::vector<PointWithObservations> point_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  int next_point_id_ = 0;                   // Unique point IDs
  
  // Keyframe parameters
  int max_keyframes_ = 8;                   // Maximum keyframes to maintain
  double keyframe_distance_threshold_ = 0.03; // 3cm minimum distance for new keyframe
  double keyframe_rotation_threshold_ = 8.0;  // 8° minimum rotation for new keyframe  
  double keyframe_feature_threshold_ = 0.7;   // Feature overlap threshold
  double match_distance_threshold_ = 0.25;    // 25cm max distance for keyframe matching
  int min_keyframe_matches_ = 15;             // Minimum matches between keyframes
  
  // Standard parameters (existing)
  double min_point_distance_ = 0.05;
  int min_observations_ = 2;
  int max_points_ = 10000;
  double outlier_radius_ = 0.2;
  int outlier_min_neighbors_ = 5;
  double voxel_leaf_size_ = 0.02;
  int publish_every_n_frames_ = 5;
  std::string world_frame_ = "world";
  std::string camera_frame_ = "camera";
  
  // RANSAC parameters  
  int ransac_max_iterations_ = 1000;
  double ransac_threshold_ = 20.0;
  double ransac_confidence_ = 0.99;
  int ransac_min_inliers_ = 5;
  
  // Preprocessing parameters
  bool use_morphological_correction_ = false;
  
  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_;
};