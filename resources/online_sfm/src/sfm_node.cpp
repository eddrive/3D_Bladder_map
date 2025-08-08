#include "online_sfm/sfm_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
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
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    RCLCPP_DEBUG(this->get_logger(), "Image converted successfully");
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

cv::Mat SfMNode::preprocessImage(const cv::Mat& raw_image) {
    RCLCPP_DEBUG(this->get_logger(), "Starting image preprocessing");
    
    // Step 1: Convert to grayscale
    cv::Mat gray_image;
    if (raw_image.channels() == 3) {
        cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = raw_image.clone();
    }
    
    // Step 2: Noise reduction with bilateral filter
    // Preserves edges while reducing noise (better than Gaussian for endoscopy)
    cv::Mat denoised;
    cv::bilateralFilter(gray_image, denoised, 9, 75, 75);
    
    // Step 3: Correct uneven illumination (common in endoscopy)
    cv::Mat illumination_corrected = correctIllumination(denoised);
    
    // Step 4: Enhance contrast adaptively
    cv::Mat contrast_enhanced = adaptiveContrastEnhancement(illumination_corrected);
    
    // Step 5: Reduce specular reflections
    cv::Mat reflection_reduced = reduceSpecularReflections(contrast_enhanced, raw_image);
    
    // Step 6: Final sharpening (subtle)
    cv::Mat sharpened = applyUnsharpMask(reflection_reduced, 1.5, 1.0);
    
    // Optional: Motion blur detection and warning
    double blur_score = detectMotionBlur(sharpened);
    if (blur_score > 0.7) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "High motion blur detected (%.2f). Consider slower movement.", blur_score);
    }
    
    // Log preprocessing statistics
    logPreprocessingStats(raw_image, sharpened);
    
    return sharpened;
}

cv::Mat SfMNode::correctIllumination(const cv::Mat& image) {
    // Method 1: Background subtraction with large Gaussian
    cv::Mat background;
    cv::GaussianBlur(image, background, cv::Size(51, 51), 0);
    
    cv::Mat corrected;
    cv::subtract(image, background, corrected);
    cv::add(corrected, cv::Scalar(128), corrected); // Add neutral gray offset
    
    // Method 2: Alternative - Top-hat morphological operation
    if (use_morphological_correction_) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        cv::Mat tophat;
        cv::morphologyEx(image, tophat, cv::MORPH_TOPHAT, kernel);
        cv::add(image, tophat, corrected);
    }
    
    return corrected;
}

cv::Mat SfMNode::adaptiveContrastEnhancement(const cv::Mat& image) {
    cv::Mat enhanced;
    
    // CLAHE (Contrast Limited Adaptive Histogram Equalization)
    // Better than global histogram equalization for medical images
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);           // Limit contrast to avoid over-enhancement
    clahe->setTilesGridSize(cv::Size(8, 8)); // Local regions for adaptation
    
    clahe->apply(image, enhanced);
    
    // Optional: Combine with original for natural look
    cv::Mat blended;
    cv::addWeighted(image, 0.7, enhanced, 0.3, 0, blended);
    
    return blended;
}

cv::Mat SfMNode::reduceSpecularReflections(const cv::Mat& gray_image, const cv::Mat& color_image) {
    // Find specular reflections (very bright pixels in color image)
    cv::Mat lab_image;
    cv::cvtColor(color_image, lab_image, cv::COLOR_BGR2Lab);
    
    std::vector<cv::Mat> lab_channels;
    cv::split(lab_image, lab_channels);
    cv::Mat lightness = lab_channels[0]; // L channel
    
    // Detect specular highlights (top 2% brightest pixels)
    cv::Mat specular_mask;
    cv::threshold(lightness, specular_mask, 240, 255, cv::THRESH_BINARY);
    
    // Dilate mask slightly to include reflection edges
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(specular_mask, specular_mask, kernel);
    
    // Inpaint specular regions using surrounding pixels
    cv::Mat result;
    cv::inpaint(gray_image, specular_mask, result, 3, cv::INPAINT_TELEA);
    
    return result;
}

cv::Mat SfMNode::applyUnsharpMask(const cv::Mat& image, double strength, double threshold) {
    // Create slightly blurred version
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(0, 0), 1.0);
    
    // Subtract blurred from original (high-pass filter)
    cv::Mat high_pass;
    cv::subtract(image, blurred, high_pass);
    
    // Apply threshold to avoid amplifying noise
    cv::Mat mask;
    cv::threshold(cv::abs(high_pass), mask, threshold, 1.0, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8UC1);
    
    // Add weighted high-pass back to original
    cv::Mat sharpened;
    cv::addWeighted(image, 1.0, high_pass, strength, 0, sharpened, CV_8UC1);
    
    return sharpened;
}

double SfMNode::detectMotionBlur(const cv::Mat& image) {
    // Use Laplacian variance to detect blur
    // Lower values = more blur
    cv::Mat laplacian;
    cv::Laplacian(image, laplacian, CV_64F);
    
    cv::Scalar mu, sigma;
    cv::meanStdDev(laplacian, mu, sigma);
    
    double variance = sigma.val[0] * sigma.val[0];
    
    // Normalize to 0-1 scale (empirically determined thresholds)
    // variance > 1000 = sharp, variance < 100 = blurry
    double blur_score = 1.0 - std::clamp(variance / 1000.0, 0.0, 1.0);
    
    return blur_score;
}

void SfMNode::logPreprocessingStats(const cv::Mat& original, const cv::Mat& processed) {
    // Calculate image statistics
    cv::Scalar original_mean = cv::mean(original);
    cv::Scalar processed_mean = cv::mean(processed);
    
    double original_std = 0, processed_std = 0;
    cv::Scalar temp_mean, temp_std;
    cv::meanStdDev(original, temp_mean, temp_std);
    original_std = temp_std.val[0];
    
    cv::meanStdDev(processed, temp_mean, temp_std);
    processed_std = temp_std.val[0];
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Preprocessing: Mean %.1f->%.1f, Std %.1f->%.1f", 
                 original_mean.val[0], processed_mean.val[0],
                 original_std, processed_std);
    
    // Log every 50 frames
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), 
                    "Image quality: contrast=%.1f, mean_brightness=%.1f", 
                    processed_std, processed_mean.val[0]);
    }
}


pcl::PointXYZ SfMNode::triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                         const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2) {
  // Normalize image coordinates
  Eigen::Vector3d ray1((kp1.pt.x - cx_) / fx_, (kp1.pt.y - cy_) / fy_, 1.0);
  Eigen::Vector3d ray2((kp2.pt.x - cx_) / fx_, (kp2.pt.y - cy_) / fy_, 1.0);
  
  // Transform rays to world coordinates
  Eigen::Matrix3d R1 = pose1.block<3, 3>(0, 0);
  Eigen::Vector3d t1 = pose1.block<3, 1>(0, 3);
  Eigen::Matrix3d R2 = pose2.block<3, 3>(0, 0);
  Eigen::Vector3d t2 = pose2.block<3, 1>(0, 3);
  
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
  
  pcl::PointXYZ point_3d = triangulatePoint(prev_kpts[match.queryIdx], 
                                             curr_kpts[match.trainIdx],
                                             prev_pose, curr_pose);
  
  // Check if triangulation was successful
  if (point_3d.x == 0 && point_3d.y == 0 && point_3d.z == 0) return false;
  
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
  
  if (ray1_world.dot(p3d - t1) <= 0 || ray2_world.dot(p3d - t2) <= 0) {
    return false;
  }
  
  // Project point back to both images and check reprojection error
  Eigen::Vector3d p1_cam = prev_pose.inverse().block<3, 3>(0, 0) * (p3d - t1);
  Eigen::Vector3d p2_cam = curr_pose.inverse().block<3, 3>(0, 0) * (p3d - t2);
  
  if (p1_cam[2] <= 0 || p2_cam[2] <= 0) return false;
  
  cv::Point2f proj1(fx_ * p1_cam[0] / p1_cam[2] + cx_, fy_ * p1_cam[1] / p1_cam[2] + cy_);
  cv::Point2f proj2(fx_ * p2_cam[0] / p2_cam[2] + cx_, fy_ * p2_cam[1] / p2_cam[2] + cy_);
  
  double error1 = cv::norm(proj1 - prev_kpts[match.queryIdx].pt);
  double error2 = cv::norm(proj2 - curr_kpts[match.trainIdx].pt);
  
  return (error1 < threshold && error2 < threshold);
}

std::vector<cv::DMatch> SfMNode::applyRANSAC(const std::vector<cv::DMatch>& matches,
                                              const std::vector<cv::KeyPoint>& prev_kpts,
                                              const std::vector<cv::KeyPoint>& curr_kpts,
                                              const Eigen::Matrix4d& prev_pose,
                                              const Eigen::Matrix4d& curr_pose) {
  if (matches.size() < ransac_min_inliers_) {
    return matches;  // Not enough matches for RANSAC
  }
  
  std::vector<cv::DMatch> best_inliers;
  int best_inlier_count = 0;
  
  int iterations = std::min(ransac_max_iterations_, 
                           static_cast<int>(matches.size() * matches.size()));
  
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
    if (current_inliers.size() > best_inlier_count) {
      best_inlier_count = current_inliers.size();
      best_inliers = current_inliers;
      
      // Early termination if we have enough inliers
      if (best_inlier_count > matches.size() * 0.8) {
        break;
      }
    }
  }
  
  RCLCPP_DEBUG(this->get_logger(), "RANSAC: %zu matches -> %d inliers", 
               matches.size(), best_inlier_count);
  
  return best_inliers.size() >= ransac_min_inliers_ ? best_inliers : std::vector<cv::DMatch>();
}

void SfMNode::processFrame(const cv::Mat& image, const Eigen::Matrix4d& T) {
  RCLCPP_DEBUG(this->get_logger(), "Processing frame of size %dx%d", image.cols, image.rows);

  // PREPROCESSING PIPELINE
    cv::Mat processed_image = preprocessImage(image);
  
  auto sift = cv::SIFT::create(1000);  // Limit to 1000 features
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift->detectAndCompute(processed_image, cv::noArray(), keypoints, descriptors);

  RCLCPP_INFO(this->get_logger(), "Frame %d: Detected %zu SIFT features", 
              frame_counter_, keypoints.size());

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
      // Apply RANSAC filtering
      std::vector<cv::DMatch> ransac_inliers = applyRANSAC(good_matches, prev_keypoints_, 
                                                            keypoints, prev_pose_, T);
      
      RCLCPP_INFO(this->get_logger(), "Frame %d: RANSAC filtered to %zu inliers", 
                  frame_counter_, ransac_inliers.size());
      
      if (!ransac_inliers.empty()) {
        // Triangulate points
        std::vector<pcl::PointXYZ> new_points;
        for (const auto& match : ransac_inliers) {
          pcl::PointXYZ point = triangulatePoint(prev_keypoints_[match.queryIdx],
                                                 keypoints[match.trainIdx],
                                                 prev_pose_, T);
          
          // Basic depth check
          Eigen::Vector3d p3d(point.x, point.y, point.z);
          Eigen::Vector3d t1 = prev_pose_.block<3, 1>(0, 3);
          Eigen::Vector3d t2 = T.block<3, 1>(0, 3);
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
  prev_pose_ = T;
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
        if (closest_idx < point_map_.size()) {
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
  if (point_map_.size() > max_points_) {
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
  if (point_map_.size() > max_points_) {
    point_map_.resize(max_points_);
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