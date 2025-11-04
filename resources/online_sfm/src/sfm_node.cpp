#include "online_sfm/sfm_node.hpp"

SfMNode::SfMNode() : Node("sfm_node")
{
  // Initialize member variables
  camera_info_received_ = false;
  is_fisheye_ = false;
  has_previous_frame_ = false;
  frame_count_ = 0;
  total_features_detected_ = 0;
  total_matches_found_ = 0;
  total_points_triangulated_ = 0;
  
  // Declare and load parameters
  declareParameters();
  loadParameters();
  
  // Initialize point cloud
  accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  // Initialize trajectory
  trajectory_.header.frame_id = world_frame_;
  
  // Setup TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create SIFT detector
  sift_detector_ = cv::SIFT::create(sift_max_features_, 
                                   sift_octave_layers_,
                                   sift_contrast_threshold_,
                                   sift_edge_threshold_,
                                   sift_sigma_);
  
  // Create CLAHE if enabled
  if (enable_clahe_) {
    clahe_ = cv::createCLAHE();
    clahe_->setClipLimit(clahe_clip_limit_);
    clahe_->setTilesGridSize(cv::Size(clahe_tile_size_, clahe_tile_size_));
  }
  
  // Setup point cloud filters
  if (enable_voxel_filtering_) {
    voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  }
  
  if (enable_outlier_filtering_) {
    outlier_filter_.setMeanK(outlier_mean_k_);
    outlier_filter_.setStddevMulThresh(outlier_std_threshold_);
  }
  
  // Create subscriptions
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/endoscope/image_raw", 10,
    std::bind(&SfMNode::imageCallback, this, std::placeholders::_1));
    
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/endoscope/camera_info", 10,
    std::bind(&SfMNode::cameraInfoCallback, this, std::placeholders::_1));
  
  // Create publishers
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sfm/point_cloud", 10);
  
  if (publish_trajectory_) {
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sfm/trajectory", 10);
  }
  
  RCLCPP_INFO(this->get_logger(), 
              "🔬 Endoscope SfM Node initialized\n"
              "   📹 Motion: baseline=%.1fmm, rotation=%.1f°\n"
              "   🔍 SIFT: %d features, contrast=%.3f, edge=%.1f\n"
              "   📊 Publishing: every %d frames to %s\n"
              "   🎯 Triangulation: depth %.1f-%.1fmm, reproj<%.1fpx\n"
              "   ☁️ Filtering: voxel=%s(%.1fmm), outlier=%s",
              min_baseline_meters_ * 1000, min_rotation_degrees_,
              sift_max_features_, sift_contrast_threshold_, sift_edge_threshold_,
              publish_every_n_frames_, world_frame_.c_str(),
              min_depth_meters_*1000, max_depth_meters_*1000, max_reprojection_error_px_,
              enable_voxel_filtering_ ? "ON" : "OFF", voxel_leaf_size_*1000,
              enable_outlier_filtering_ ? "ON" : "OFF");
}

void SfMNode::declareParameters()
{
  // Motion thresholds
  this->declare_parameter("motion.min_baseline_meters", 0.003);
  this->declare_parameter("motion.min_rotation_degrees", 1.0);
  
  // SIFT parameters
  this->declare_parameter("sift.max_features", 3000);
  this->declare_parameter("sift.octave_layers", 4);
  this->declare_parameter("sift.contrast_threshold", 0.02);
  this->declare_parameter("sift.edge_threshold", 8.0);
  this->declare_parameter("sift.sigma", 1.6);
  
  // Image processing
  this->declare_parameter("image_processing.enable_clahe", true);
  this->declare_parameter("image_processing.clahe_clip_limit", 2.5);
  this->declare_parameter("image_processing.clahe_tile_size", 8);
  this->declare_parameter("image_processing.enable_sharpening", false);
  this->declare_parameter("image_processing.sharpening_strength", 0.3);
  
  // Matching parameters
  this->declare_parameter("matching.lowe_ratio_threshold", 0.8);
  this->declare_parameter("matching.min_matches_for_triangulation", 30);
  this->declare_parameter("matching.enable_cross_check", false);
  
  // Triangulation validation
  this->declare_parameter("triangulation.min_angle_degrees", 1.0);
  this->declare_parameter("triangulation.max_angle_degrees", 179.0);
  this->declare_parameter("triangulation.max_reprojection_error_px", 8.0);
  this->declare_parameter("triangulation.min_depth_meters", 0.005);
  this->declare_parameter("triangulation.max_depth_meters", 0.5);
  this->declare_parameter("triangulation.max_triangulation_error_meters", 0.03);
  this->declare_parameter("triangulation.image_margin_px", 30);
  
  // Point cloud filtering
  this->declare_parameter("point_cloud.enable_voxel_filtering", true);
  this->declare_parameter("point_cloud.voxel_leaf_size", 0.002);
  this->declare_parameter("point_cloud.enable_outlier_filtering", true);
  this->declare_parameter("point_cloud.outlier_mean_k", 20);
  this->declare_parameter("point_cloud.outlier_std_threshold", 2.0);
  
  // Publishing
  this->declare_parameter("publishing.world_frame", "world");
  this->declare_parameter("publishing.camera_frame", "camera");
  this->declare_parameter("publishing.publish_every_n_frames", 1);
  this->declare_parameter("publishing.publish_trajectory", true);
  
  // Diagnostics
  this->declare_parameter("diagnostics.enable_detailed_logging", true);
  this->declare_parameter("diagnostics.enable_image_diagnostics", false);
  this->declare_parameter("diagnostics.log_every_n_frames", 5);
}

void SfMNode::loadParameters()
{
  // Motion thresholds
  min_baseline_meters_ = this->get_parameter("motion.min_baseline_meters").as_double();
  min_rotation_degrees_ = this->get_parameter("motion.min_rotation_degrees").as_double();
  
  // SIFT parameters
  sift_max_features_ = this->get_parameter("sift.max_features").as_int();
  sift_octave_layers_ = this->get_parameter("sift.octave_layers").as_int();
  sift_contrast_threshold_ = this->get_parameter("sift.contrast_threshold").as_double();
  sift_edge_threshold_ = this->get_parameter("sift.edge_threshold").as_double();
  sift_sigma_ = this->get_parameter("sift.sigma").as_double();
  
  // Image processing
  enable_clahe_ = this->get_parameter("image_processing.enable_clahe").as_bool();
  clahe_clip_limit_ = this->get_parameter("image_processing.clahe_clip_limit").as_double();
  clahe_tile_size_ = this->get_parameter("image_processing.clahe_tile_size").as_int();
  enable_sharpening_ = this->get_parameter("image_processing.enable_sharpening").as_bool();
  sharpening_strength_ = this->get_parameter("image_processing.sharpening_strength").as_double();
  
  // Matching parameters
  lowe_ratio_threshold_ = this->get_parameter("matching.lowe_ratio_threshold").as_double();
  min_matches_for_triangulation_ = this->get_parameter("matching.min_matches_for_triangulation").as_int();
  enable_cross_check_ = this->get_parameter("matching.enable_cross_check").as_bool();
  
  // Triangulation validation
  min_triangulation_angle_deg_ = this->get_parameter("triangulation.min_angle_degrees").as_double();
  max_triangulation_angle_deg_ = this->get_parameter("triangulation.max_angle_degrees").as_double();
  max_reprojection_error_px_ = this->get_parameter("triangulation.max_reprojection_error_px").as_double();
  min_depth_meters_ = this->get_parameter("triangulation.min_depth_meters").as_double();
  max_depth_meters_ = this->get_parameter("triangulation.max_depth_meters").as_double();
  max_triangulation_error_meters_ = this->get_parameter("triangulation.max_triangulation_error_meters").as_double();
  image_margin_px_ = this->get_parameter("triangulation.image_margin_px").as_int();
  
  // Point cloud filtering
  enable_voxel_filtering_ = this->get_parameter("point_cloud.enable_voxel_filtering").as_bool();
  voxel_leaf_size_ = this->get_parameter("point_cloud.voxel_leaf_size").as_double();
  enable_outlier_filtering_ = this->get_parameter("point_cloud.enable_outlier_filtering").as_bool();
  outlier_mean_k_ = this->get_parameter("point_cloud.outlier_mean_k").as_int();
  outlier_std_threshold_ = this->get_parameter("point_cloud.outlier_std_threshold").as_double();
  
  // Publishing
  world_frame_ = this->get_parameter("publishing.world_frame").as_string();
  camera_frame_ = this->get_parameter("publishing.camera_frame").as_string();
  publish_every_n_frames_ = this->get_parameter("publishing.publish_every_n_frames").as_int();
  publish_trajectory_ = this->get_parameter("publishing.publish_trajectory").as_bool();
  
  // Diagnostics
  enable_detailed_logging_ = this->get_parameter("diagnostics.enable_detailed_logging").as_bool();
  enable_image_diagnostics_ = this->get_parameter("diagnostics.enable_image_diagnostics").as_bool();
  log_every_n_frames_ = this->get_parameter("diagnostics.log_every_n_frames").as_int();
}

void SfMNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (camera_info_received_) return;
  
  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat(msg->d).clone();
  
  fx_ = camera_matrix_.at<double>(0, 0);
  fy_ = camera_matrix_.at<double>(1, 1);
  cx_ = camera_matrix_.at<double>(0, 2);
  cy_ = camera_matrix_.at<double>(1, 2);
  
  is_fisheye_ = (msg->distortion_model == "fisheye");
  camera_info_received_ = true;
  
  RCLCPP_INFO(this->get_logger(), 
              "📷 Camera calibrated: %s model\n"
              "   🔧 Intrinsics: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f\n"
              "   🔍 Distortion: %zu coefficients", 
              is_fisheye_ ? "FISHEYE" : "STANDARD", 
              fx_, fy_, cx_, cy_, msg->d.size());
}

bool SfMNode::getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(world_frame_, camera_frame_, stamp,
                                                rclcpp::Duration::from_nanoseconds(100000000));
    
    Eigen::Translation3d translation(transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   transform.transform.translation.z);
    
    Eigen::Quaterniond rotation(transform.transform.rotation.w,
                               transform.transform.rotation.x,
                               transform.transform.rotation.y,
                               transform.transform.rotation.z);
    
    pose = (translation * rotation).matrix();
    return true;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                          "Failed to get camera pose: %s", ex.what());
    return false;
  }
}

bool SfMNode::shouldProcessFrame(const Eigen::Matrix4d& current_pose)
{
  if (!has_previous_frame_) {
    return true;
  }
  
  // Check baseline
  Eigen::Vector3d prev_pos = previous_pose_.block<3,1>(0,3);
  Eigen::Vector3d curr_pos = current_pose.block<3,1>(0,3);
  double baseline = (curr_pos - prev_pos).norm();
  
  // Check rotation
  Eigen::Matrix3d prev_rot = previous_pose_.block<3,3>(0,0);
  Eigen::Matrix3d curr_rot = current_pose.block<3,3>(0,0);
  Eigen::Matrix3d delta_rot = prev_rot.transpose() * curr_rot;
  Eigen::AngleAxisd angle_axis(delta_rot);
  double rotation_degrees = std::abs(angle_axis.angle()) * 180.0 / M_PI;
  
  bool sufficient_baseline = baseline >= min_baseline_meters_;
  bool sufficient_rotation = rotation_degrees >= min_rotation_degrees_;
  
  if (sufficient_baseline || sufficient_rotation) {
    if (enable_detailed_logging_ && (frame_count_ % log_every_n_frames_ == 0)) {
      RCLCPP_INFO(this->get_logger(), 
                  "✅ Keyframe triggered: baseline=%.1fmm, rotation=%.1f°",
                  baseline * 1000, rotation_degrees);
    }
    return true;
  }
  
  return false;
}

void SfMNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!camera_info_received_) {
    return;
  }
  
  // Get camera pose
  Eigen::Matrix4d current_pose;
  if (!getCameraPose(msg->header.stamp, current_pose)) {
    return;
  }
  
  // Check if we should process this frame
  if (!shouldProcessFrame(current_pose)) {
    return;
  }
  
  // Convert image
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }
  
  frame_count_++;
  
  if (enable_detailed_logging_ && (frame_count_ % log_every_n_frames_ == 0)) {
    RCLCPP_INFO(this->get_logger(), "🔬 Processing frame %d", frame_count_);
  }
  
  processFrame(image, msg->header.stamp);
  
  // Update trajectory
  if (publish_trajectory_) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = world_frame_;
    
    Eigen::Vector3d position = current_pose.block<3,1>(0,3);
    Eigen::Quaterniond quaternion(current_pose.block<3,3>(0,0));
    
    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();
    pose_msg.pose.orientation.w = quaternion.w();
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    
    trajectory_.poses.push_back(pose_msg);
    trajectory_.header.stamp = msg->header.stamp;
  }
  
  // Save current state for next frame
  previous_pose_ = current_pose;
  previous_timestamp_ = msg->header.stamp;
}

void SfMNode::processFrame(const cv::Mat& image, const rclcpp::Time& /*timestamp*/)
{
  // Preprocess image
  cv::Mat processed_image = preprocessImage(image);
  
  // Detect features
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectFeatures(processed_image, keypoints, descriptors);
  
  total_features_detected_ += keypoints.size();
  
  if (enable_image_diagnostics_) {
    logDiagnostics(image, processed_image, keypoints);
  }
  
  int triangulated_points = 0;
  
  // If we have a previous frame, perform matching and triangulation
  if (has_previous_frame_ && !previous_descriptors_.empty() && keypoints.size() > 50) {
    
    // Match features
    std::vector<cv::DMatch> matches = matchFeatures(previous_descriptors_, descriptors);
    total_matches_found_ += matches.size();
    
    if (matches.size() >= static_cast<size_t>(min_matches_for_triangulation_)) {
      
      // Filter matches
      std::vector<cv::DMatch> good_matches = filterMatches(matches, previous_keypoints_, keypoints);
      
      if (good_matches.size() >= static_cast<size_t>(min_matches_for_triangulation_)) {
        
        // Triangulate points
        triangulateAndAddPoints(good_matches, previous_keypoints_, keypoints,
                               previous_pose_, previous_pose_,
                               previous_image_, processed_image);
        
        triangulated_points = good_matches.size(); // Approximate
        total_points_triangulated_ += triangulated_points;
        
        // Publish point cloud if needed
        if (frame_count_ % publish_every_n_frames_ == 0) {
          publishPointCloud();
          if (publish_trajectory_) {
            publishTrajectory();
          }
        }
      }
    }
  }
  
  // Log statistics
  if (enable_detailed_logging_ && (frame_count_ % log_every_n_frames_ == 0)) {
    logFrameStatistics(frame_count_, keypoints.size(), 
                      has_previous_frame_ ? static_cast<int>(total_matches_found_) : 0, 
                      triangulated_points);
  }
  
  // Save current frame data
  previous_image_ = processed_image.clone();
  previous_keypoints_ = keypoints;
  previous_descriptors_ = descriptors.clone();
  has_previous_frame_ = true;
}

cv::Mat SfMNode::preprocessImage(const cv::Mat& input)
{
  cv::Mat processed;
  
  // Convert to grayscale if needed
  if (input.channels() == 3) {
    cv::cvtColor(input, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = input.clone();
  }
  
  // Undistort
  processed = undistortImage(processed);
  
  // Apply CLAHE if enabled
  if (enable_clahe_ && clahe_) {
    cv::Mat clahe_result;
    clahe_->apply(processed, clahe_result);
    processed = clahe_result;
  }
  
  // Apply sharpening if enabled
  if (enable_sharpening_) {
    cv::Mat kernel = (cv::Mat_<float>(3,3) << 
                      0, -sharpening_strength_, 0,
                      -sharpening_strength_, 1 + 4*sharpening_strength_, -sharpening_strength_,
                      0, -sharpening_strength_, 0);
    cv::Mat sharpened;
    cv::filter2D(processed, sharpened, -1, kernel);
    processed = sharpened;
  }
  
  return processed;
}

cv::Mat SfMNode::undistortImage(const cv::Mat& input)
{
  cv::Mat undistorted;
  
  if (is_fisheye_) {
    cv::fisheye::undistortImage(input, undistorted, camera_matrix_, dist_coeffs_, camera_matrix_);
  } else {
    cv::undistort(input, undistorted, camera_matrix_, dist_coeffs_, camera_matrix_);
  }
  
  return undistorted;
}

void SfMNode::detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
  sift_detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}

std::vector<cv::DMatch> SfMNode::matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2)
{
  if (desc1.empty() || desc2.empty()) {
    return std::vector<cv::DMatch>();
  }
  
  cv::BFMatcher matcher(cv::NORM_L2, enable_cross_check_);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  
  if (enable_cross_check_) {
    std::vector<cv::DMatch> matches;
    matcher.match(desc1, desc2, matches);
    return matches;
  } else {
    matcher.knnMatch(desc1, desc2, knn_matches, 2);
    
    std::vector<cv::DMatch> good_matches;
    for (const auto& match_pair : knn_matches) {
      if (match_pair.size() == 2 && 
          match_pair[0].distance < lowe_ratio_threshold_ * match_pair[1].distance) {
        good_matches.push_back(match_pair[0]);
      }
    }
    return good_matches;
  }
}

std::vector<cv::DMatch> SfMNode::filterMatches(const std::vector<cv::DMatch>& matches,
                                               const std::vector<cv::KeyPoint>& kp1,
                                               const std::vector<cv::KeyPoint>& kp2)
{
  std::vector<cv::DMatch> filtered_matches;
  
  for (const auto& match : matches) {
    const cv::KeyPoint& keypoint1 = kp1[match.queryIdx];
    const cv::KeyPoint& keypoint2 = kp2[match.trainIdx];
    
    // Check if keypoints are within image margins
    if (keypoint1.pt.x >= image_margin_px_ && 
        keypoint1.pt.x < (1920 - image_margin_px_) &&
        keypoint1.pt.y >= image_margin_px_ && 
        keypoint1.pt.y < (1080 - image_margin_px_) &&
        keypoint2.pt.x >= image_margin_px_ && 
        keypoint2.pt.x < (1920 - image_margin_px_) &&
        keypoint2.pt.y >= image_margin_px_ && 
        keypoint2.pt.y < (1080 - image_margin_px_)) {
      
      filtered_matches.push_back(match);
    }
  }
  
  return filtered_matches;
}

void SfMNode::triangulateAndAddPoints(const std::vector<cv::DMatch>& matches,
                                     const std::vector<cv::KeyPoint>& kp1,
                                     const std::vector<cv::KeyPoint>& kp2,
                                     const Eigen::Matrix4d& pose1,
                                     const Eigen::Matrix4d& pose2,
                                     const cv::Mat& /*image1*/,
                                     const cv::Mat& image2)
{
  int successful_triangulations = 0;
  
  for (const auto& match : matches) {
    pcl::PointXYZRGB point = triangulatePoint(kp1[match.queryIdx], kp2[match.trainIdx],
                                             pose1, pose2, image2);
    
    if (point.x != 0 || point.y != 0 || point.z != 0) {  // Valid point
      if (validateTriangulatedPoint(point, kp1[match.queryIdx], kp2[match.trainIdx], pose1, pose2)) {
        addPointToCloud(point);
        successful_triangulations++;
      }
    }
  }
  
  if (enable_detailed_logging_ && successful_triangulations > 0) {
    RCLCPP_INFO(this->get_logger(), 
                "✅ Triangulated %d/%zu points (total cloud: %zu)",
                successful_triangulations, matches.size(), accumulated_cloud_->size());
  }
}

pcl::PointXYZRGB SfMNode::triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                          const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2,
                                          const cv::Mat& image2)
{
  pcl::PointXYZRGB invalid_point;
  invalid_point.x = invalid_point.y = invalid_point.z = 0;
  invalid_point.r = invalid_point.g = invalid_point.b = 0;
  
  // Get camera positions and rotations
  Eigen::Vector3d t1 = pose1.block<3, 1>(0, 3);
  Eigen::Vector3d t2 = pose2.block<3, 1>(0, 3);
  Eigen::Matrix3d R1 = pose1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = pose2.block<3, 3>(0, 0);
  
  // Check baseline
  double baseline = (t2 - t1).norm();
  if (baseline < min_baseline_meters_) {
    return invalid_point;
  }
  
  // Convert pixel coordinates to normalized camera coordinates
  Eigen::Vector3d ray1_cam((kp1.pt.x - cx_) / fx_, (kp1.pt.y - cy_) / fy_, 1.0);
  Eigen::Vector3d ray2_cam((kp2.pt.x - cx_) / fx_, (kp2.pt.y - cy_) / fy_, 1.0);
  
  // Transform to world coordinates
  Eigen::Vector3d ray1_world = R1 * ray1_cam;
  Eigen::Vector3d ray2_world = R2 * ray2_cam;
  
  // Check triangulation angle
  double ray_angle = std::acos(std::clamp(ray1_world.dot(ray2_world), -1.0, 1.0));
  double ray_angle_degrees = ray_angle * 180.0 / M_PI;
  
  if (ray_angle_degrees < min_triangulation_angle_deg_ || 
      ray_angle_degrees > max_triangulation_angle_deg_) {
    return invalid_point;
  }
  
  // Triangulate using least squares
  Eigen::Vector3d w = t1 - t2;
  double a = ray1_world.dot(ray1_world);
  double b = ray1_world.dot(ray2_world);
  double c = ray2_world.dot(ray2_world);
  double d = ray1_world.dot(w);
  double e = ray2_world.dot(w);
  
  double denom = a * c - b * b;
  if (std::abs(denom) < 1e-10) {
    return invalid_point;
  }
  
  double s = (b * e - c * d) / denom;
  double t = (a * e - b * d) / denom;
  
  // Check depth positivity
  if (s <= min_depth_meters_ || t <= min_depth_meters_) {
    return invalid_point;
  }
  
  // Calculate 3D point
  Eigen::Vector3d point1 = t1 + s * ray1_world;
  Eigen::Vector3d point2 = t2 + t * ray2_world;
  Eigen::Vector3d point_3d = 0.5 * (point1 + point2);
  
  // Check triangulation error
  double triangulation_error = (point1 - point2).norm();
  if (triangulation_error > max_triangulation_error_meters_) {
    return invalid_point;
  }
  
  // Check depth range
  double distance_cam1 = (point_3d - t1).norm();
  if (distance_cam1 < min_depth_meters_ || distance_cam1 > max_depth_meters_) {
    return invalid_point;
  }
  
  // Create colored point
  pcl::PointXYZRGB colored_point;
  colored_point.x = point_3d.x();
  colored_point.y = point_3d.y();
  colored_point.z = point_3d.z();
  
  // Extract color from the second image (current frame)
  extractColor(colored_point, kp2, image2);
  
  return colored_point;
}

bool SfMNode::validateTriangulatedPoint(const pcl::PointXYZRGB& point,
                                        const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                        const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2)
{
  // Reproject point to both cameras and check reprojection error
  Eigen::Vector3d point_3d(point.x, point.y, point.z);
  
  // Camera 1 reprojection
  Eigen::Vector3d t1 = pose1.block<3, 1>(0, 3);
  Eigen::Matrix3d R1 = pose1.block<3, 3>(0, 0);
  Eigen::Vector3d point_cam1 = R1.transpose() * (point_3d - t1);
  
  if (point_cam1.z() <= 0) return false;
  
  double u1_proj = fx_ * (point_cam1.x() / point_cam1.z()) + cx_;
  double v1_proj = fy_ * (point_cam1.y() / point_cam1.z()) + cy_;
  double error1 = std::sqrt(std::pow(u1_proj - kp1.pt.x, 2) + std::pow(v1_proj - kp1.pt.y, 2));
  
  // Camera 2 reprojection
  Eigen::Vector3d t2 = pose2.block<3, 1>(0, 3);
  Eigen::Matrix3d R2 = pose2.block<3, 3>(0, 0);
  Eigen::Vector3d point_cam2 = R2.transpose() * (point_3d - t2);
  
  if (point_cam2.z() <= 0) return false;
  
  double u2_proj = fx_ * (point_cam2.x() / point_cam2.z()) + cx_;
  double v2_proj = fy_ * (point_cam2.y() / point_cam2.z()) + cy_;
  double error2 = std::sqrt(std::pow(u2_proj - kp2.pt.x, 2) + std::pow(v2_proj - kp2.pt.y, 2));
  
  return (error1 <= max_reprojection_error_px_ && error2 <= max_reprojection_error_px_);
}

void SfMNode::addPointToCloud(const pcl::PointXYZRGB& point)
{
  accumulated_cloud_->push_back(point);
  
  // Apply filtering periodically to keep cloud manageable
  if (accumulated_cloud_->size() % 1000 == 0) {
    filterPointCloud();
  }
}

void SfMNode::filterPointCloud()
{
  if (accumulated_cloud_->empty()) return;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  // Apply voxel grid filtering
  if (enable_voxel_filtering_) {
    voxel_filter_.setInputCloud(accumulated_cloud_);
    voxel_filter_.filter(*filtered_cloud);
  } else {
    *filtered_cloud = *accumulated_cloud_;
  }
  
  // Apply statistical outlier removal
  if (enable_outlier_filtering_ && filtered_cloud->size() > static_cast<size_t>(outlier_mean_k_)) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    outlier_filter_.setInputCloud(filtered_cloud);
    outlier_filter_.filter(*final_cloud);
    accumulated_cloud_ = final_cloud;
  } else {
    accumulated_cloud_ = filtered_cloud;
  }
}

void SfMNode::publishPointCloud()
{
  if (accumulated_cloud_->empty()) return;
  
  accumulated_cloud_->width = accumulated_cloud_->points.size();
  accumulated_cloud_->height = 1;
  accumulated_cloud_->is_dense = false;
  
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*accumulated_cloud_, cloud_msg);
  cloud_msg.header.frame_id = world_frame_;
  cloud_msg.header.stamp = this->now();
  
  pointcloud_pub_->publish(cloud_msg);
  
  if (enable_detailed_logging_) {
    RCLCPP_DEBUG(this->get_logger(), "📤 Published point cloud with %zu points", accumulated_cloud_->size());
  }
}

void SfMNode::publishTrajectory()
{
  if (trajectory_.poses.empty()) return;
  
  trajectory_.header.stamp = this->now();
  trajectory_pub_->publish(trajectory_);
}

void SfMNode::extractColor(pcl::PointXYZRGB& point, const cv::KeyPoint& kp, const cv::Mat& image)
{
  int x = static_cast<int>(std::round(kp.pt.x));
  int y = static_cast<int>(std::round(kp.pt.y));
  
  if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
    if (image.channels() == 3) {
      cv::Vec3b color = image.at<cv::Vec3b>(y, x);
      point.b = color[0];
      point.g = color[1];
      point.r = color[2];
    } else {
      uint8_t gray = image.at<uint8_t>(y, x);
      point.r = point.g = point.b = gray;
    }
  } else {
    // Default to white if pixel is out of bounds
    point.r = point.g = point.b = 255;
  }
}

void SfMNode::logFrameStatistics(int frame_count, int features, int matches, int triangulated)
{
  double avg_features = static_cast<double>(total_features_detected_) / frame_count;
  double avg_matches = has_previous_frame_ ? static_cast<double>(total_matches_found_) / (frame_count - 1) : 0;
  
  RCLCPP_INFO(this->get_logger(),
              "📊 Frame %d Stats:\n"
              "   🔍 Features: %d (avg: %.1f)\n"
              "   🔗 Matches: %d (avg: %.1f)\n"
              "   📍 Triangulated: %d\n"
              "   ☁️ Total points: %zu",
              frame_count, features, avg_features, matches, avg_matches, 
              triangulated, accumulated_cloud_->size());
}

void SfMNode::logDiagnostics(const cv::Mat& raw_image, const cv::Mat& processed_image,
                            const std::vector<cv::KeyPoint>& keypoints)
{
  // Raw image analysis
  cv::Mat gray_raw;
  if (raw_image.channels() == 3) {
    cv::cvtColor(raw_image, gray_raw, cv::COLOR_BGR2GRAY);
  } else {
    gray_raw = raw_image.clone();
  }
  
  cv::Scalar mean_raw, stddev_raw;
  cv::meanStdDev(gray_raw, mean_raw, stddev_raw);
  
  // Processed image analysis
  cv::Scalar mean_proc, stddev_proc;
  cv::meanStdDev(processed_image, mean_proc, stddev_proc);
  
  // Feature distribution analysis
  int center_features = 0, border_features = 0;
  int center_x = processed_image.cols / 2;
  int center_y = processed_image.rows / 2;
  int radius = std::min(processed_image.cols, processed_image.rows) / 4;
  
  for (const auto& kp : keypoints) {
    double dist = std::sqrt(std::pow(kp.pt.x - center_x, 2) + std::pow(kp.pt.y - center_y, 2));
    if (dist < radius) {
      center_features++;
    } else {
      border_features++;
    }
  }
  
  RCLCPP_INFO(this->get_logger(),
              "🔍 Image Diagnostics:\n"
              "   📊 Raw: brightness=%.1f, contrast=%.1f\n"
              "   📊 Processed: brightness=%.1f, contrast=%.1f\n"
              "   🎯 Features: %zu total, %d center, %d border\n"
              "   📈 Center/Border ratio: %.2f",
              mean_raw[0], stddev_raw[0], mean_proc[0], stddev_proc[0],
              keypoints.size(), center_features, border_features,
              border_features > 0 ? static_cast<double>(center_features) / border_features : 0.0);
}

// Main function
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<SfMNode>();
  
  RCLCPP_INFO(node->get_logger(), "🚀 SfM Node started");
  
  rclcpp::spin(node);
  
  RCLCPP_INFO(node->get_logger(), "🛑 SfM Node stopped");
  rclcpp::shutdown();
  
  return 0;
}