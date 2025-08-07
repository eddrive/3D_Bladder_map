#include "online_sfm/sfm_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <numeric>

SfMNode::SfMNode() : Node("online_sfm"), rng_(std::random_device{}()), uniform_dist_(0.0, 1.0) {
  // QoS settings for better compatibility
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/endoscope/image_raw", qos,
    std::bind(&SfMNode::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/endoscope/camera_info", qos,
    std::bind(&SfMNode::cameraInfoCallback, this, std::placeholders::_1));

  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sfm/point_cloud", 10);
  filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sfm/filtered_point_cloud", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  
  // Declare parameters with defaults
  this->declare_parameter("min_point_distance", 0.05);
  this->declare_parameter("min_observations", 2);
  this->declare_parameter("max_points", 10000);
  this->declare_parameter("publish_every_n_frames", 5);
  this->declare_parameter("ransac_threshold", 3.0);
  this->declare_parameter("ransac_min_inliers", 20);
  this->declare_parameter("world_frame", "world");
  this->declare_parameter("camera_frame", "camera");
  
  // Get parameters
  min_point_distance_ = this->get_parameter("min_point_distance").as_double();
  min_observations_ = this->get_parameter("min_observations").as_int();
  max_points_ = this->get_parameter("max_points").as_int();
  publish_every_n_frames_ = this->get_parameter("publish_every_n_frames").as_int();
  ransac_threshold_ = this->get_parameter("ransac_threshold").as_double();
  ransac_min_inliers_ = this->get_parameter("ransac_min_inliers").as_int();
  
  RCLCPP_INFO(this->get_logger(), "RANSAC params: threshold=%.1f, min_inliers=%d", 
              ransac_threshold_, ransac_min_inliers_);
  world_frame_ = this->get_parameter("world_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  
  RCLCPP_INFO(this->get_logger(), "Using TF frames: %s -> %s", 
              world_frame_.c_str(), camera_frame_.c_str());
              
  RCLCPP_INFO(this->get_logger(), "SfM Node initialized and ready to receive images");
}

void SfMNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (cam_info_received_) return;
  
  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat(msg->d).clone();
  
  fx_ = camera_matrix_.at<double>(0, 0);
  fy_ = camera_matrix_.at<double>(1, 1);
  cx_ = camera_matrix_.at<double>(0, 2);
  cy_ = camera_matrix_.at<double>(1, 2);
  
  cam_info_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera info received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
              fx_, fy_, cx_, cy_);
}

bool SfMNode::getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam) {
  try {
    // First try: exact timestamp with short timeout
    auto tf_msg = tf_buffer_->lookupTransform(world_frame_, camera_frame_, stamp, 
                                              rclcpp::Duration::from_nanoseconds(50000000)); // 50ms timeout
    
    Eigen::Translation3d t(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);
    Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                         tf_msg.transform.rotation.x,
                         tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    T_world_cam = (t * q).matrix();
    
    RCLCPP_DEBUG(this->get_logger(), "Got TF at exact timestamp");
    return true;
    
  } catch (const tf2::TransformException& e) {
    RCLCPP_DEBUG(this->get_logger(), "Exact TF failed: %s", e.what());
    
    // Second try: latest available transform
    try {
      auto tf_msg = tf_buffer_->lookupTransform(world_frame_, camera_frame_, tf2::TimePointZero);
      
      Eigen::Translation3d t(tf_msg.transform.translation.x,
                             tf_msg.transform.translation.y,
                             tf_msg.transform.translation.z);
      Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                           tf_msg.transform.rotation.x,
                           tf_msg.transform.rotation.y,
                           tf_msg.transform.rotation.z);
      T_world_cam = (t * q).matrix();
      
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Using latest TF instead of timestamp %f", stamp.seconds());
      return true;
      
    } catch (const tf2::TransformException& e2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "No TF available at all (%s -> %s): %s", 
                            world_frame_.c_str(), camera_frame_.c_str(), e2.what());
      return false;
    }
  }
}

void SfMNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Image received: %dx%d, encoding: %s", 
                        msg->width, msg->height, msg->encoding.c_str());
  
  if (!cam_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Camera info not received yet");
    return;
  }

  cv::Mat image;
  try {
    // Try to handle both rgb8 and bgr8 encodings
    if (msg->encoding == "rgb8") {
      image = cv_bridge::toCvShare(msg, "rgb8")->image;
      cv::cvtColor(image, image, cv::COLOR_RGB2BGR);  // Convert to BGR for OpenCV
    } else {
      image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    RCLCPP_DEBUG(this->get_logger(), "Image converted successfully from %s", msg->encoding.c_str());
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  Eigen::Matrix4d pose;
  if (!getCameraPose(msg->header.stamp, pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Cannot get camera pose, skipping frame");
    return;
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Processing frame %d", frame_counter_);

  processFrame(image, pose);
  frame_counter_++;
}

pcl::PointXYZ SfMNode::triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                         const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2) {
  // Normalize image coordinates
  Eigen::Vector3d ray1((kp1.pt.x - cx_) / fx_, (kp1.pt.y - cy_) / fy_, 1.0);
  Eigen::Vector3d ray2((kp2.pt.x - cx_) / fx_, (kp2.pt.y - cy_) / fy_, 1.0);
  
  // Transform rays to world coordinates
  Eigen::Matrix3d R1 = pose1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = pose2.block<3, 3>(0, 0);
  
  Eigen::Vector3d ray1_world = R1 * ray1;
  Eigen::Vector3d ray2_world = R2 * ray2;

  // DLT triangulation
  Eigen::MatrixXd A(4, 4);
  A.row(0) = ray1_world[0] * pose1.row(2) - pose1.row(0);
  A.row(1) = ray1_world[1] * pose1.row(2) - pose1.row(1);
  A.row(2) = ray2_world[0] * pose2.row(2) - pose2.row(0);
  A.row(3) = ray2_world[1] * pose2.row(2) - pose2.row(1);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d point_homogeneous = svd.matrixV().col(3);
  
  if (std::abs(point_homogeneous[3]) < 1e-10) {
    return pcl::PointXYZ(0, 0, 0);  // Invalid point
  }
  
  Eigen::Vector3d point_3d = point_homogeneous.head<3>() / point_homogeneous[3];
  return pcl::PointXYZ(point_3d.x(), point_3d.y(), point_3d.z());
}

bool SfMNode::isInlier(const cv::DMatch& match,
                       const std::vector<cv::KeyPoint>& prev_kpts,
                       const std::vector<cv::KeyPoint>& curr_kpts,
                       const Eigen::Matrix4d& prev_pose,
                       const Eigen::Matrix4d& curr_pose,
                       double threshold) {
  
  // Reset debug counter periodically and debug first match of each frame
  static int debug_count = 0;
  static int last_frame = -1;
  
  // Reset counter every new frame (use frame counter somehow, or just reset every 100 calls)
  debug_count++;
  if (debug_count > 100) debug_count = 0;
  
  bool debug_this = debug_count <= 1;  // Debug only first match
  
  if (debug_this) {
    RCLCPP_INFO(this->get_logger(), "=== DEBUGGING MATCH (call %d) ===", debug_count);
  }
  
  pcl::PointXYZ point_3d = triangulatePoint(prev_kpts[match.queryIdx], 
                                             curr_kpts[match.trainIdx],
                                             prev_pose, curr_pose);
  
  // Check if triangulation was successful
  if (point_3d.x == 0 && point_3d.y == 0 && point_3d.z == 0) {
    if (debug_this) RCLCPP_WARN(this->get_logger(), "❌ Triangulation failed for match");
    return false;
  }
  
  if (debug_this) {
    RCLCPP_INFO(this->get_logger(), "✅ Triangulated point: [%.3f, %.3f, %.3f]", 
                point_3d.x, point_3d.y, point_3d.z);
    
    // Show the keypoint coordinates too
    RCLCPP_INFO(this->get_logger(), "   Keypoint 1: (%.1f, %.1f)", 
                prev_kpts[match.queryIdx].pt.x, prev_kpts[match.queryIdx].pt.y);
    RCLCPP_INFO(this->get_logger(), "   Keypoint 2: (%.1f, %.1f)", 
                curr_kpts[match.trainIdx].pt.x, curr_kpts[match.trainIdx].pt.y);
  }
  
  // Check if point is in front of both cameras
  Eigen::Vector3d p3d(point_3d.x, point_3d.y, point_3d.z);
  Eigen::Vector3d t1 = prev_pose.block<3, 1>(0, 3);
  Eigen::Vector3d t2 = curr_pose.block<3, 1>(0, 3);
  
  Eigen::Vector3d ray1_world = prev_pose.block<3, 3>(0, 0) * 
    Eigen::Vector3d((prev_kpts[match.queryIdx].pt.x - cx_) / fx_, 
                    (prev_kpts[match.queryIdx].pt.y - cy_) / fy_, 1.0);
  Eigen::Vector3d ray2_world = curr_pose.block<3, 3>(0, 0) * 
    Eigen::Vector3d((curr_kpts[match.trainIdx].pt.x - cx_) / fx_, 
                    (curr_kpts[match.trainIdx].pt.y - cy_) / fy_, 1.0);
  
  double depth1 = ray1_world.dot(p3d - t1);
  double depth2 = ray2_world.dot(p3d - t2);
  
  if (debug_this) {
    RCLCPP_INFO(this->get_logger(), "   Depths: cam1=%.3f, cam2=%.3f", depth1, depth2);
    RCLCPP_INFO(this->get_logger(), "   Camera positions: t1=[%.3f,%.3f,%.3f], t2=[%.3f,%.3f,%.3f]", 
                t1.x(), t1.y(), t1.z(), t2.x(), t2.y(), t2.z());
    RCLCPP_INFO(this->get_logger(), "   Camera intrinsics: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                fx_, fy_, cx_, cy_);
  }
  
  if (depth1 <= 0 || depth2 <= 0) {
    if (debug_this) RCLCPP_WARN(this->get_logger(), "❌ Point behind camera(s) - depth1=%.3f, depth2=%.3f", depth1, depth2);
    return false;
  }
  
  // Project point back to both images and check reprojection error
  Eigen::Vector3d p1_cam = prev_pose.inverse().block<3, 3>(0, 0) * (p3d - t1);
  Eigen::Vector3d p2_cam = curr_pose.inverse().block<3, 3>(0, 0) * (p3d - t2);
  
  if (debug_this) {
    RCLCPP_INFO(this->get_logger(), "   Points in camera coords: p1_cam=[%.3f,%.3f,%.3f], p2_cam=[%.3f,%.3f,%.3f]",
                p1_cam.x(), p1_cam.y(), p1_cam.z(), p2_cam.x(), p2_cam.y(), p2_cam.z());
  }
  
  if (p1_cam[2] <= 0 || p2_cam[2] <= 0) {
    if (debug_this) RCLCPP_WARN(this->get_logger(), "❌ Negative depth after transformation: p1z=%.3f, p2z=%.3f", p1_cam[2], p2_cam[2]);
    return false;
  }
  
  cv::Point2f proj1(fx_ * p1_cam[0] / p1_cam[2] + cx_, fy_ * p1_cam[1] / p1_cam[2] + cy_);
  cv::Point2f proj2(fx_ * p2_cam[0] / p2_cam[2] + cx_, fy_ * p2_cam[1] / p2_cam[2] + cy_);
  
  double error1 = cv::norm(proj1 - prev_kpts[match.queryIdx].pt);
  double error2 = cv::norm(proj2 - curr_kpts[match.trainIdx].pt);
  
  if (debug_this) {
    RCLCPP_INFO(this->get_logger(), "   Reprojection errors: %.2f, %.2f (threshold=%.2f)", 
                error1, error2, threshold);
    RCLCPP_INFO(this->get_logger(), "   Original pts: (%.1f,%.1f), (%.1f,%.1f)", 
                prev_kpts[match.queryIdx].pt.x, prev_kpts[match.queryIdx].pt.y,
                curr_kpts[match.trainIdx].pt.x, curr_kpts[match.trainIdx].pt.y);
    RCLCPP_INFO(this->get_logger(), "   Projected pts: (%.1f,%.1f), (%.1f,%.1f)", 
                proj1.x, proj1.y, proj2.x, proj2.y);
  }
  
  bool is_inlier = (error1 < threshold && error2 < threshold);
  if (debug_this) {
    if (is_inlier) {
      RCLCPP_INFO(this->get_logger(), "✅ Match result: INLIER");
    } else {
      RCLCPP_WARN(this->get_logger(), "❌ Match result: OUTLIER (error1=%.2f > %.2f OR error2=%.2f > %.2f)", 
                  error1, threshold, error2, threshold);
    }
    RCLCPP_INFO(this->get_logger(), "=== END DEBUGGING MATCH ===");
  }
  
  return is_inlier;
}

std::vector<cv::DMatch> SfMNode::applyRANSAC(const std::vector<cv::DMatch>& matches,
                                              const std::vector<cv::KeyPoint>& prev_kpts,
                                              const std::vector<cv::KeyPoint>& curr_kpts,
                                              const Eigen::Matrix4d& prev_pose,
                                              const Eigen::Matrix4d& curr_pose) {
  if (matches.size() < static_cast<size_t>(ransac_min_inliers_)) {
    RCLCPP_WARN(this->get_logger(), "RANSAC: Not enough matches (%zu < %d)", 
                matches.size(), ransac_min_inliers_);
    return matches;  // Not enough matches for RANSAC
  }
  
  std::vector<cv::DMatch> best_inliers;
  int best_inlier_count = 0;
  
  int iterations = std::min(ransac_max_iterations_, 
                           static_cast<int>(matches.size() * matches.size()));
  
  RCLCPP_DEBUG(this->get_logger(), "RANSAC: Starting with %zu matches, %d iterations", 
               matches.size(), iterations);
  
  // Test a few matches manually for debugging
  int manual_inliers = 0;
  RCLCPP_INFO(this->get_logger(), "=== MANUAL INLIER TEST ===");
  for (size_t i = 0; i < std::min(matches.size(), size_t(5)); i += 1) {
    bool is_inlier_result = isInlier(matches[i], prev_kpts, curr_kpts, prev_pose, curr_pose, ransac_threshold_);
    if (is_inlier_result) {
      manual_inliers++;
    }
    RCLCPP_INFO(this->get_logger(), "Match %zu: %s", i, is_inlier_result ? "INLIER" : "OUTLIER");
  }
  RCLCPP_INFO(this->get_logger(), "=== END MANUAL TEST ===");
  RCLCPP_INFO(this->get_logger(), "RANSAC Debug: %d/%d sample matches are inliers", 
              manual_inliers, std::min(int(matches.size()), 5));
  
  for (int iter = 0; iter < iterations; ++iter) {
    // Randomly sample matches
    std::vector<int> sample_indices;
    std::unordered_set<int> used_indices;
    
    while (sample_indices.size() < 5 && used_indices.size() < matches.size()) {
      int idx = static_cast<int>(uniform_dist_(rng_) * matches.size());
      if (used_indices.find(idx) == used_indices.end()) {
        used_indices.insert(idx);
        sample_indices.push_back(idx);
      }
    }
    
    // Count inliers for this sample
    std::vector<cv::DMatch> current_inliers;
    for (const auto& match : matches) {
      if (isInlier(match, prev_kpts, curr_kpts, prev_pose, curr_pose, ransac_threshold_)) {
        current_inliers.push_back(match);
      }
    }
    
    // Update best model if this one is better
    if (static_cast<int>(current_inliers.size()) > best_inlier_count) {
      best_inlier_count = static_cast<int>(current_inliers.size());
      best_inliers = current_inliers;
      
      // Early termination if we have enough inliers
      if (best_inlier_count > matches.size() * 0.8) {
        break;
      }
    }
    
    // Report progress every 100 iterations
    if (iter % 100 == 0 && iter > 0) {
      RCLCPP_DEBUG(this->get_logger(), "RANSAC iter %d: best inliers = %d", iter, best_inlier_count);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "RANSAC: %zu matches -> %d inliers (threshold=%.1f)", 
              matches.size(), best_inlier_count, ransac_threshold_);
  
  return best_inliers.size() >= static_cast<size_t>(ransac_min_inliers_) ? best_inliers : std::vector<cv::DMatch>();
}

void SfMNode::processFrame(const cv::Mat& image, const Eigen::Matrix4d& T) {
  RCLCPP_DEBUG(this->get_logger(), "Processing frame of size %dx%d", image.cols, image.rows);
  
  // Make a mutable copy for potential modification
  Eigen::Matrix4d current_pose = T;
  
  // Check image statistics
  cv::Scalar mean_val = cv::mean(image);
  double min_val, max_val;
  cv::minMaxLoc(image, &min_val, &max_val);
  
  RCLCPP_INFO(this->get_logger(), "Image stats - Mean: %.1f, Min: %.1f, Max: %.1f", 
              mean_val[0], min_val, max_val);
  
  // Convert to grayscale for feature detection
  cv::Mat gray_image;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  } else {
    gray_image = image.clone();
  }
  
  // Apply histogram equalization to enhance contrast
  cv::Mat enhanced_image;
  cv::equalizeHist(gray_image, enhanced_image);
  
  // More aggressive SIFT parameters
  auto sift = cv::SIFT::create(
    2000,      // nfeatures - increase from 1000
    4,         // nOctaveLayers 
    0.03,      // contrastThreshold - lower = more features (default 0.04)
    5,         // edgeThreshold - lower = more features (default 10)
    1.6        // sigma
  );
  
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  
  // Try on both original and enhanced images
  sift->detectAndCompute(gray_image, cv::noArray(), keypoints, descriptors);
  
  RCLCPP_INFO(this->get_logger(), "Frame %d: Detected %zu SIFT features on original image", 
              frame_counter_, keypoints.size());
  
  // If no features on original, try enhanced
  if (keypoints.empty()) {
    sift->detectAndCompute(enhanced_image, cv::noArray(), keypoints, descriptors);
    RCLCPP_INFO(this->get_logger(), "Frame %d: Detected %zu SIFT features on enhanced image", 
                frame_counter_, keypoints.size());
  }
  
  // If still no features, try ORB as fallback
  if (keypoints.empty()) {
    RCLCPP_WARN(this->get_logger(), "No SIFT features found, trying ORB...");
    auto orb = cv::ORB::create(2000);
    orb->detectAndCompute(enhanced_image, cv::noArray(), keypoints, descriptors);
    RCLCPP_INFO(this->get_logger(), "Frame %d: Detected %zu ORB features", 
                frame_counter_, keypoints.size());
  }

  if (has_prev_ && !prev_descriptors_.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Matching with previous frame");
    
    // Match features with ratio test
    std::vector<std::vector<cv::DMatch>> knn_matches;
    auto matcher = cv::BFMatcher(cv::NORM_L2);
    
    try {
      matcher.knnMatch(prev_descriptors_, descriptors, knn_matches, 2);
      RCLCPP_DEBUG(this->get_logger(), "KNN matching completed: %zu matches", knn_matches.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "KNN matching failed: %s", e.what());
      goto update_frame_data;
    }

    // Apply Lowe's ratio test
    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (const auto& match_pair : knn_matches) {
      if (match_pair.size() >= 2 && 
          match_pair[0].distance < ratio_thresh * match_pair[1].distance) {
        good_matches.push_back(match_pair[0]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Frame %d: %zu raw matches -> %zu good matches", 
                frame_counter_, knn_matches.size(), good_matches.size());

    if (good_matches.size() >= 10) {
      // Debug: Print both poses
      Eigen::Vector3d t1 = prev_pose_.block<3, 1>(0, 3);
      Eigen::Vector3d t2 = current_pose.block<3, 1>(0, 3);
      
      RCLCPP_INFO(this->get_logger(), "Pose1: [%.3f, %.3f, %.3f]", t1.x(), t1.y(), t1.z());
      RCLCPP_INFO(this->get_logger(), "Pose2: [%.3f, %.3f, %.3f]", t2.x(), t2.y(), t2.z());
      
      double translation_distance = (t2 - t1).norm();
      
      Eigen::Matrix3d R1 = prev_pose_.block<3, 3>(0, 0);
      Eigen::Matrix3d R2 = current_pose.block<3, 3>(0, 0);
      Eigen::Matrix3d R_diff = R2 * R1.transpose();
      double angle_diff = std::acos(std::clamp((R_diff.trace() - 1) / 2, -1.0, 1.0));
      
      RCLCPP_INFO(this->get_logger(), "Frame %d: Camera moved %.4fm, rotated %.2f°", 
                  frame_counter_, translation_distance, angle_diff * 180.0 / M_PI);
      
      // Force visual odometry for testing - DISABLED, use TF directly
      if (false && translation_distance < 0.050) {  // DISABLED - use TF poses directly
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "TF shows no movement, trying visual odometry estimation...");
        
        // Use OpenCV's essential matrix to estimate camera motion
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& match : good_matches) {
          pts1.push_back(prev_keypoints_[match.queryIdx].pt);
          pts2.push_back(keypoints[match.trainIdx].pt);
        }
        
        // Estimate essential matrix
        cv::Mat essential_matrix = cv::findEssentialMat(
          pts1, pts2, 
          camera_matrix_, 
          cv::RANSAC, 0.999, 1.0
        );
        
        if (!essential_matrix.empty()) {
          cv::Mat R, t, mask;
          int inliers = cv::recoverPose(essential_matrix, pts1, pts2, camera_matrix_, R, t, mask);
          
          if (inliers > 10) {
            RCLCPP_INFO(this->get_logger(), 
                        "Visual odometry: translation=%.3fm, %d inliers", 
                        cv::norm(t), inliers);
            
            // Create new pose based on visual odometry
            Eigen::Matrix3d R_eigen;
            Eigen::Vector3d t_eigen;
            cv::cv2eigen(R, R_eigen);
            cv::cv2eigen(t, t_eigen);
            
            // Scale translation (assume movement is around 1-5cm)
            double scale = 0.02;  // 2cm typical movement
            t_eigen *= scale;
            
            Eigen::Matrix4d estimated_pose = prev_pose_;
            estimated_pose.block<3, 3>(0, 0) = prev_pose_.block<3, 3>(0, 0) * R_eigen;
            estimated_pose.block<3, 1>(0, 3) += prev_pose_.block<3, 3>(0, 0) * t_eigen;
            
            // Use estimated pose instead of TF
            current_pose = estimated_pose;
            
            RCLCPP_INFO(this->get_logger(), 
                        "Using estimated pose with scale %.3fm", scale);
          }
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Sufficient movement detected, proceeding with triangulation");
      }
      
      // Apply RANSAC filtering
      std::vector<cv::DMatch> ransac_inliers = applyRANSAC(good_matches, prev_keypoints_, 
                                                            keypoints, prev_pose_, current_pose);
      
      RCLCPP_INFO(this->get_logger(), "Frame %d: RANSAC filtered to %zu inliers", 
                  frame_counter_, ransac_inliers.size());
      
      if (!ransac_inliers.empty()) {
        // Triangulate points
        std::vector<pcl::PointXYZ> new_points;
        for (const auto& match : ransac_inliers) {
          pcl::PointXYZ point = triangulatePoint(prev_keypoints_[match.queryIdx],
                                                 keypoints[match.trainIdx],
                                                 prev_pose_, current_pose);
          
          // Basic depth check
          Eigen::Vector3d p3d(point.x, point.y, point.z);
          Eigen::Vector3d t1 = prev_pose_.block<3, 1>(0, 3);
          Eigen::Vector3d t2 = current_pose.block<3, 1>(0, 3);
          double depth1 = (p3d - t1).norm();
          double depth2 = (p3d - t2).norm();
          
          if (depth1 > 0.1 && depth1 < 50.0 && depth2 > 0.1 && depth2 < 50.0) {
            new_points.push_back(point);
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "Frame %d: Triangulated %zu valid points", 
                    frame_counter_, new_points.size());
        
        // Add points to accumulated map
        if (!new_points.empty()) {
          addPointsToMap(new_points, ransac_inliers, keypoints);
          RCLCPP_INFO(this->get_logger(), "Frame %d: Added %zu points (total: %zu)", 
                      frame_counter_, new_points.size(), point_map_.size());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Frame %d: No inliers found after RANSAC", frame_counter_);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Frame %d: Insufficient good matches (%zu < 10)", 
                  frame_counter_, good_matches.size());
    }

    // Publish accumulated point cloud periodically
    if (frame_counter_ % publish_every_n_frames_ == 0) {
      publishAccumulatedPointCloud();
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Frame %d: First frame or no previous descriptors", frame_counter_);
  }

update_frame_data:
  prev_image_ = image.clone();
  prev_descriptors_ = descriptors.clone();
  prev_keypoints_ = keypoints;
  prev_pose_ = current_pose;
  has_prev_ = true;
  
  RCLCPP_DEBUG(this->get_logger(), "Frame data updated for next iteration");
}

void SfMNode::addPointsToMap(const std::vector<pcl::PointXYZ>& new_points,
                             const std::vector<cv::DMatch>& matches,
                             const std::vector<cv::KeyPoint>& current_keypoints) {
  
  // Build KD-tree from existing points for efficient nearest neighbor search
  if (!accumulated_cloud_->empty()) {
    kdtree_.setInputCloud(accumulated_cloud_);
  }
  
  for (size_t i = 0; i < new_points.size(); ++i) {
    const auto& point = new_points[i];
    bool is_duplicate = false;
    
    // Check for nearby points to avoid duplicates
    if (!accumulated_cloud_->empty()) {
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      
      if (kdtree_.radiusSearch(point, min_point_distance_, 
                               neighbor_indices, neighbor_distances) > 0) {
        // Found nearby points, merge with the closest one
        int closest_idx = neighbor_indices[0];
        if (closest_idx < static_cast<int>(point_map_.size())) {
          point_map_[closest_idx].observation_count++;
          point_map_[closest_idx].frame_ids.push_back(frame_counter_);
          point_map_[closest_idx].observations.push_back(current_keypoints[matches[i].trainIdx]);
          is_duplicate = true;
        }
      }
    }
    
    // Add as new point if not duplicate
    if (!is_duplicate) {
      PointWithObservations new_point_obs;
      new_point_obs.point = point;
      new_point_obs.frame_ids.push_back(frame_counter_);
      new_point_obs.observations.push_back(current_keypoints[matches[i].trainIdx]);
      new_point_obs.observation_count = 1;
      new_point_obs.quality_score = calculatePointQuality(point, {current_keypoints[matches[i].trainIdx]});
      
      point_map_.push_back(new_point_obs);
      accumulated_cloud_->push_back(point);
    }
  }
  
  // Remove points with too few observations or maintain max points limit
  if (point_map_.size() > static_cast<size_t>(max_points_)) {
    filterPointCloud();
  }
}

double SfMNode::calculatePointQuality(const pcl::PointXYZ& point, 
                                       const std::vector<cv::KeyPoint>& observations) {
  double quality = 0.0;
  
  // Factor 1: Number of observations
  quality += observations.size() * 0.5;
  
  // Factor 2: Average keypoint response (feature strength)
  double avg_response = 0.0;
  for (const auto& kp : observations) {
    avg_response += kp.response;
  }
  quality += (avg_response / observations.size()) * 0.3;
  
  // Factor 3: Distance from camera (prefer points at reasonable depth)
  double depth = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
  if (depth > 1.0 && depth < 10.0) {
    quality += 0.2;
  }
  
  return quality;
}

void SfMNode::filterPointCloud() {
  RCLCPP_INFO(this->get_logger(), "Filtering point cloud: %zu -> ", point_map_.size());
  
  // Sort by quality and keep the best points
  std::sort(point_map_.begin(), point_map_.end(),
            [](const PointWithObservations& a, const PointWithObservations& b) {
              return a.quality_score > b.quality_score;
            });
  
  // Remove points with insufficient observations
  auto it = std::remove_if(point_map_.begin(), point_map_.end(),
                           [this](const PointWithObservations& p) {
                             return p.observation_count < min_observations_;
                           });
  point_map_.erase(it, point_map_.end());
  
  // Limit to max points
  if (point_map_.size() > static_cast<size_t>(max_points_)) {
    point_map_.resize(static_cast<size_t>(max_points_));
  }
  
  // Rebuild accumulated cloud
  accumulated_cloud_->clear();
  for (const auto& point_obs : point_map_) {
    accumulated_cloud_->push_back(point_obs.point);
  }
  
  RCLCPP_INFO(this->get_logger(), "%zu points after filtering", point_map_.size());
}

void SfMNode::publishAccumulatedPointCloud() {
  if (accumulated_cloud_->empty()) return;
  
  // Publish raw accumulated cloud
  accumulated_cloud_->width = accumulated_cloud_->points.size();
  accumulated_cloud_->height = 1;
  accumulated_cloud_->is_dense = true;
  
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*accumulated_cloud_, cloud_msg);
  cloud_msg.header.frame_id = world_frame_;
  cloud_msg.header.stamp = this->now();
  pc_pub_->publish(cloud_msg);
  
  // Apply additional filtering for the filtered cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  
  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(accumulated_cloud_);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(*filtered_cloud);
  
  // Radius outlier removal
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
  outlier_filter.setInputCloud(filtered_cloud);
  outlier_filter.setRadiusSearch(outlier_radius_);
  outlier_filter.setMinNeighborsInRadius(outlier_min_neighbors_);
  outlier_filter.filter(*filtered_cloud);
  
  // Publish filtered cloud
  if (!filtered_cloud->empty()) {
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header.frame_id = world_frame_;
    filtered_msg.header.stamp = this->now();
    filtered_pc_pub_->publish(filtered_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published clouds: raw=%zu, filtered=%zu points",
                accumulated_cloud_->size(), filtered_cloud->size());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SfMNode>());
  rclcpp::shutdown();
  return 0;
}