#include "online_sfm/sfm_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>

// ===============================================
// COSTRUTTORE E SETUP
// ===============================================

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
  
  // PARAMETRI ENDOSCOPI
  this->declare_parameter("min_baseline_meters", 0.015);
  this->declare_parameter("min_rotation_degrees", 1.5);
  this->declare_parameter("max_time_between_keyframes", 20.0);
  this->declare_parameter("ransac_threshold", 8.0);
  this->declare_parameter("ransac_min_inliers", 12);
  this->declare_parameter("world_frame", "world");
  this->declare_parameter("camera_frame", "camera");
  
  min_baseline_meters_ = this->get_parameter("min_baseline_meters").as_double();
  min_rotation_degrees_ = this->get_parameter("min_rotation_degrees").as_double();
  max_time_between_keyframes_ = this->get_parameter("max_time_between_keyframes").as_double();
  ransac_threshold_ = this->get_parameter("ransac_threshold").as_double();
  ransac_min_inliers_ = this->get_parameter("ransac_min_inliers").as_int();
  world_frame_ = this->get_parameter("world_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  
  RCLCPP_INFO(this->get_logger(), 
              "🔬 ENDOSCOPE SfM will trigger when baseline > %.3fm OR rotation > %.1f° OR time > %.1fs", 
              min_baseline_meters_, min_rotation_degrees_, max_time_between_keyframes_);
  RCLCPP_INFO(this->get_logger(), "Using TF frames: %s -> %s", 
              world_frame_.c_str(), camera_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "🔬 Endoscope SfM Node initialized and ready to receive images");
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
  
  RCLCPP_INFO(this->get_logger(), "🔬 ENDOSCOPE calibration loaded:");
  RCLCPP_INFO(this->get_logger(), "   fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
  RCLCPP_INFO(this->get_logger(), "   Distortion: k1=%.3f, k2=%.3f (fisheye-like)", 
              msg->d[0], msg->d[1]);
}

bool SfMNode::getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam) {
  try {
    // TF: world → camera (Z uscente, X destra, Y giù)
    auto tf_msg = tf_buffer_->lookupTransform(world_frame_, camera_frame_, stamp, 
                                              rclcpp::Duration::from_nanoseconds(50000000));
    
    Eigen::Translation3d t(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);
    Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                         tf_msg.transform.rotation.x,
                         tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    
    // Matrice TF (ROS convention)
    Eigen::Matrix4d T_world_cam_ros = (t * q).matrix();
    
    // 🔧 CONVERSIONE ROS → OPENCV
    // ROS camera: Z uscente, X destra, Y giù
    // OpenCV:     Z entrante, X destra, Y giù
    // Soluzione: Ruota di 180° attorno all'asse X
    
    Eigen::Matrix4d ros_to_opencv = Eigen::Matrix4d::Identity();
    ros_to_opencv(1, 1) = -1;  // Flip Y
    ros_to_opencv(2, 2) = -1;  // Flip Z
    
    // Applica conversione: T_opencv = T_ros * R_ros_to_opencv
    T_world_cam = T_world_cam_ros * ros_to_opencv;
    
    // Debug ogni tanto
    static int debug_counter = 0;
    if (++debug_counter % 10 == 0) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "🔧 ROS→OpenCV conversion applied - Z flipped for triangulation");
    }
    
    return true;
    
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                          "Cannot get camera pose (%s -> %s): %s", 
                          world_frame_.c_str(), camera_frame_.c_str(), e.what());
    return false;
  }
}

// ===============================================
// CALLBACKS E KEYFRAME LOGIC
// ===============================================

bool SfMNode::shouldProcessFrame(const Eigen::Matrix4d& current_pose, 
                                 const rclcpp::Time& current_time) {
  
  if (!has_prev_) {
    RCLCPP_INFO(this->get_logger(), "First frame - processing as keyframe");
    return true;
  }
  
  // Calcola differenze spaziali
  Eigen::Vector3d t_prev = prev_pose_.block<3,1>(0,3);
  Eigen::Vector3d t_curr = current_pose.block<3,1>(0,3);
  double baseline = (t_curr - t_prev).norm();
  
  // Calcola differenza angolare
  Eigen::Matrix3d R_prev = prev_pose_.block<3,3>(0,0);
  Eigen::Matrix3d R_curr = current_pose.block<3,3>(0,0);
  Eigen::Matrix3d dR = R_prev.transpose() * R_curr;
  Eigen::AngleAxisd angle_diff(dR);
  double angle_degrees = std::abs(angle_diff.angle() * 180.0 / M_PI);
  
  // Calcola differenza temporale
  double time_diff = (current_time - last_keyframe_time_).seconds();
  
  // Criteri per processare il frame
  bool baseline_sufficient = baseline > min_baseline_meters_;
  bool rotation_sufficient = angle_degrees > min_rotation_degrees_;
  bool time_expired = time_diff > max_time_between_keyframes_;
  
  if (baseline_sufficient || rotation_sufficient || time_expired) {
    std::string reason = baseline_sufficient ? "baseline" : 
                        (rotation_sufficient ? "rotation" : "timeout");
    
    RCLCPP_INFO(this->get_logger(), 
                "🎯 KEYFRAME TRIGGER (%s) - Baseline: %.4fm, Rotation: %.2f°, Time: %.2fs", 
                reason.c_str(), baseline, angle_degrees, time_diff);
    return true;
  }
  
  return false;
}

void SfMNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Image received: %dx%d, encoding: %s", 
                        msg->width, msg->height, msg->encoding.c_str());
  
  if (!cam_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Camera info not received yet");
    return;
  }

  Eigen::Matrix4d current_pose;
  if (!getCameraPose(msg->header.stamp, current_pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                          "Cannot get camera pose, skipping frame");
    return;
  }
  
  // CONTROLLA SE DOBBIAMO PROCESSARE QUESTO FRAME
  if (!shouldProcessFrame(current_pose, msg->header.stamp)) {
    current_image_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    current_pose_ = current_pose;
    frame_counter_++;
    return;
  }
  
  // PROCESSA IL KEYFRAME
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "🔥 Processing keyframe %d", keyframe_counter_);
  
  processFrame(image, current_pose);
  
  // Aggiorna i dati del keyframe precedente
  prev_image_ = image.clone();
  prev_pose_ = current_pose;
  last_keyframe_time_ = msg->header.stamp;
  has_prev_ = true;
  keyframe_counter_++;
  frame_counter_++;
}

// ===============================================
// FUNZIONI SPECIFICHE ENDOSCOPI  
// ===============================================

std::vector<cv::KeyPoint> SfMNode::undistortKeypoints(const std::vector<cv::KeyPoint>& keypoints) {
  if (keypoints.empty()) return keypoints;
  
  std::vector<cv::Point2f> points_distorted, points_undistorted;
  for (const auto& kp : keypoints) {
    points_distorted.push_back(kp.pt);
  }
  
  cv::undistortPoints(points_distorted, points_undistorted, camera_matrix_, dist_coeffs_, 
                      cv::noArray(), camera_matrix_);
  
  std::vector<cv::KeyPoint> undistorted_keypoints;
  for (size_t i = 0; i < keypoints.size(); ++i) {
    cv::KeyPoint kp = keypoints[i];
    kp.pt = points_undistorted[i];
    undistorted_keypoints.push_back(kp);
  }
  
  static int undist_debug = 0;
  if (++undist_debug % 200 == 0) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "🔬 Undistortion sample: (%.1f,%.1f) -> (%.1f,%.1f)",
                 points_distorted[0].x, points_distorted[0].y,
                 points_undistorted[0].x, points_undistorted[0].y);
  }
  
  return undistorted_keypoints;
}

bool SfMNode::isInlierUndistorted(const cv::DMatch& match,
                                  const std::vector<cv::KeyPoint>& prev_kpts_undist,
                                  const std::vector<cv::KeyPoint>& curr_kpts_undist,
                                  const Eigen::Matrix4d& prev_pose,
                                  const Eigen::Matrix4d& curr_pose,
                                  double threshold) {
  
  static int debug_counter = 0;
  bool is_debug_match = (++debug_counter % 50 == 0); // Debug ogni 50 matches per non intasare i log
  
  // Step 1: Controlli indici
  if (static_cast<size_t>(match.queryIdx) >= prev_kpts_undist.size() || 
      static_cast<size_t>(match.trainIdx) >= curr_kpts_undist.size()) {
    if (is_debug_match) {
      RCLCPP_ERROR(this->get_logger(), "🚨 RANSAC DEBUG: Invalid indices %d/%zu, %d/%zu", 
                   match.queryIdx, prev_kpts_undist.size(), match.trainIdx, curr_kpts_undist.size());
    }
    return false;
  }
  
  const cv::KeyPoint& kp1 = prev_kpts_undist[match.queryIdx];
  const cv::KeyPoint& kp2 = curr_kpts_undist[match.trainIdx];
  
  if (is_debug_match) {
    RCLCPP_INFO(this->get_logger(), "🔍 RANSAC DEBUG Step 1: Match points (%.1f,%.1f) -> (%.1f,%.1f), distance=%.2f", 
                kp1.pt.x, kp1.pt.y, kp2.pt.x, kp2.pt.y, match.distance);
  }
  
  // Step 2: Triangolazione
  pcl::PointXYZ point_3d = triangulatePoint(kp1, kp2, prev_pose, curr_pose);
  
  if (std::abs(point_3d.x) < 1e-6 && std::abs(point_3d.y) < 1e-6 && std::abs(point_3d.z) < 1e-6) {
    if (is_debug_match) {
      RCLCPP_WARN(this->get_logger(), "🚨 RANSAC DEBUG Step 2: Triangulation FAILED - point at origin");
    }
    return false;
  }
  
  if (is_debug_match) {
    RCLCPP_INFO(this->get_logger(), "🔍 RANSAC DEBUG Step 2: Triangulated point (%.3f, %.3f, %.3f)", 
                point_3d.x, point_3d.y, point_3d.z);
  }
  
  // Step 3: Controllo depth
  double depth = std::sqrt(point_3d.x*point_3d.x + point_3d.y*point_3d.y + point_3d.z*point_3d.z);
  if (depth < 0.005 || depth > 10.0) {
    if (is_debug_match) {
      RCLCPP_WARN(this->get_logger(), "🚨 RANSAC DEBUG Step 3: Invalid depth %.3fm (range: 0.005-10.0m)", depth);
    }
    return false;
  }
  
  if (is_debug_match) {
    RCLCPP_INFO(this->get_logger(), "🔍 RANSAC DEBUG Step 3: Depth OK: %.3fm", depth);
  }
  
  // Step 4: Controllo posizione camere
  Eigen::Vector4d point_homogeneous(point_3d.x, point_3d.y, point_3d.z, 1.0);
  
  Eigen::Matrix4d T_cam1_world = prev_pose.inverse();
  Eigen::Matrix4d T_cam2_world = curr_pose.inverse();
  
  Eigen::Vector4d point_cam1 = T_cam1_world * point_homogeneous;
  Eigen::Vector4d point_cam2 = T_cam2_world * point_homogeneous;
  
  if (point_cam1.z() <= 0.005 || point_cam2.z() <= 0.005) {
    if (is_debug_match) {
      RCLCPP_WARN(this->get_logger(), "🚨 RANSAC DEBUG Step 4: Point behind camera(s) - z1=%.3f, z2=%.3f (min: 0.005)", 
                  point_cam1.z(), point_cam2.z());
    }
    return false;
  }
  
  if (is_debug_match) {
    RCLCPP_INFO(this->get_logger(), "🔍 RANSAC DEBUG Step 4: Camera depths OK - z1=%.3f, z2=%.3f", 
                point_cam1.z(), point_cam2.z());
  }
  
  // Step 5: Proiezione
  Eigen::Vector3d projected_1 = projectToCamera(point_homogeneous, prev_pose);
  Eigen::Vector3d projected_2 = projectToCamera(point_homogeneous, curr_pose);
  
  // Controlla proiezioni valide
  if ((projected_1.x() < 0 && projected_1.y() < 0) || 
      (projected_2.x() < 0 && projected_2.y() < 0)) {
    if (is_debug_match) {
      RCLCPP_WARN(this->get_logger(), "🚨 RANSAC DEBUG Step 5: Invalid projection - proj1(%.1f,%.1f), proj2(%.1f,%.1f)", 
                  projected_1.x(), projected_1.y(), projected_2.x(), projected_2.y());
    }
    return false;
  }
  
  if (is_debug_match) {
    RCLCPP_INFO(this->get_logger(), "🔍 RANSAC DEBUG Step 5: Projections OK - proj1(%.1f,%.1f), proj2(%.1f,%.1f)", 
                projected_1.x(), projected_1.y(), projected_2.x(), projected_2.y());
  }
  
  // Step 6: Calcolo errori reproiezione
  double error_1 = std::sqrt(std::pow(projected_1.x() - kp1.pt.x, 2) + 
                             std::pow(projected_1.y() - kp1.pt.y, 2));
  double error_2 = std::sqrt(std::pow(projected_2.x() - kp2.pt.x, 2) + 
                             std::pow(projected_2.y() - kp2.pt.y, 2));
  
  bool is_inlier = (error_1 < threshold && error_2 < threshold);
  
  if (is_debug_match) {
    RCLCPP_INFO(this->get_logger(), 
                "🔍 RANSAC DEBUG Step 6: FINAL RESULT\n"
                "   Original: kp1(%.1f,%.1f), kp2(%.1f,%.1f)\n"
                "   Projected: proj1(%.1f,%.1f), proj2(%.1f,%.1f)\n"
                "   Errors: %.2f, %.2f pixels (threshold: %.2f)\n"
                "   Result: %s", 
                kp1.pt.x, kp1.pt.y, kp2.pt.x, kp2.pt.y,
                projected_1.x(), projected_1.y(), projected_2.x(), projected_2.y(),
                error_1, error_2, threshold,
                is_inlier ? "✅ INLIER" : "❌ OUTLIER");
  }
  
  return is_inlier;
}

bool SfMNode::isInlier(const cv::DMatch& match,
                       const std::vector<cv::KeyPoint>& prev_kpts,
                       const std::vector<cv::KeyPoint>& curr_kpts,
                       const Eigen::Matrix4d& prev_pose,
                       const Eigen::Matrix4d& curr_pose,
                       double threshold) {
  
  if (static_cast<size_t>(match.queryIdx) >= prev_kpts.size() || 
      static_cast<size_t>(match.trainIdx) >= curr_kpts.size()) {
    return false;
  }
  
  const cv::KeyPoint& kp1 = prev_kpts[match.queryIdx];
  const cv::KeyPoint& kp2 = curr_kpts[match.trainIdx];
  
  pcl::PointXYZ point_3d = triangulatePoint(kp1, kp2, prev_pose, curr_pose);
  
  if (std::abs(point_3d.x) < 1e-6 && std::abs(point_3d.y) < 1e-6 && std::abs(point_3d.z) < 1e-6) {
    return false;
  }
  
  double depth = std::sqrt(point_3d.x*point_3d.x + point_3d.y*point_3d.y + point_3d.z*point_3d.z);
  if (depth < 0.01 || depth > 100.0) {
    return false;
  }
  
  Eigen::Vector4d point_homogeneous(point_3d.x, point_3d.y, point_3d.z, 1.0);
  
  Eigen::Vector3d projected_1 = projectToCamera(point_homogeneous, prev_pose);
  Eigen::Vector3d projected_2 = projectToCamera(point_homogeneous, curr_pose);
  
  if (projected_1.x() < 0 && projected_1.y() < 0) return false;
  if (projected_2.x() < 0 && projected_2.y() < 0) return false;
  
  double error_1 = std::sqrt(std::pow(projected_1.x() - kp1.pt.x, 2) + 
                             std::pow(projected_1.y() - kp1.pt.y, 2));
  double error_2 = std::sqrt(std::pow(projected_2.x() - kp2.pt.x, 2) + 
                             std::pow(projected_2.y() - kp2.pt.y, 2));
  
  return (error_1 < threshold && error_2 < threshold);
}

// ===============================================
// PROCESS FRAME - CUORE DEL SISTEMA
// ===============================================


void SfMNode::processFrame(const cv::Mat& image, const Eigen::Matrix4d& T) {
  RCLCPP_INFO(this->get_logger(), "🔬 Processing endoscope keyframe of size %dx%d", image.cols, image.rows);

  // Preprocessing
  cv::Mat processed_image = preprocessImage(image);
  
  // Feature detection
  auto sift = cv::SIFT::create(1500);
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift->detectAndCompute(processed_image, cv::noArray(), keypoints, descriptors);

  RCLCPP_INFO(this->get_logger(), "🔬 Keyframe %d: Detected %zu SIFT features", 
              keyframe_counter_, keypoints.size());

  if (has_prev_ && !prev_descriptors_.empty()) {
    // Calcola baseline
    Eigen::Vector3d t1 = prev_pose_.block<3,1>(0,3);
    Eigen::Vector3d t2 = T.block<3,1>(0,3);
    double baseline = (t2 - t1).norm();
    
    // DEFINISCI keyframe_threshold QUI per essere disponibile in tutto lo scope
    double keyframe_threshold = baseline > 0.03 ? 5.0 : 4.0;
    
    RCLCPP_INFO(this->get_logger(), "🔬 Endoscope keyframe pair baseline: %.4fm", baseline);
    
    // 🔍 DEBUG POSE DETTAGLIATO
    debugCameraPoses(prev_pose_, T, keyframe_counter_);
    
    // 🔧 CORREZIONE DISTORSIONE per endoscopi!
    std::vector<cv::KeyPoint> undistorted_keypoints = undistortKeypoints(keypoints);
    std::vector<cv::KeyPoint> undistorted_prev_keypoints = undistortKeypoints(prev_keypoints_);
    
    RCLCPP_DEBUG(this->get_logger(), "🔬 Applied distortion correction to keypoints");
    
    // Match features
    std::vector<std::vector<cv::DMatch>> knn_matches;
    auto matcher = cv::BFMatcher(cv::NORM_L2);
    
    try {
      matcher.knnMatch(prev_descriptors_, descriptors, knn_matches, 2);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "🔬 KNN matching failed: %s", e.what());
      goto update_keyframe_data;
    }

    // Lowe's ratio test (più permissivo per endoscopi)
    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = baseline > 0.03 ? 0.85f : 0.8f;
    
    for (const auto& match_pair : knn_matches) {
      if (match_pair.size() >= 2 && 
          match_pair[0].distance < ratio_thresh * match_pair[1].distance) {
        good_matches.push_back(match_pair[0]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "🔬 Keyframe %d: %zu raw matches -> %zu good matches", 
                keyframe_counter_, knn_matches.size(), good_matches.size());

    if (good_matches.size() >= 12) { // Meno restrittivo per endoscopi
      
      RCLCPP_INFO(this->get_logger(), "🔬🔍 RANSAC DEBUG START - Processing %zu good matches with threshold %.1f", 
                  good_matches.size(), keyframe_threshold);
      
      std::vector<cv::DMatch> ransac_inliers;
      for (const auto& match : good_matches) {
        // USA KEYPOINTS NON-DISTORTI per RANSAC!
        if (isInlierUndistorted(match, undistorted_prev_keypoints, undistorted_keypoints, 
                               prev_pose_, T, keyframe_threshold)) {
          ransac_inliers.push_back(match);
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "🔬 Keyframe %d: %zu good matches -> %zu RANSAC inliers (threshold: %.1f)", 
                  keyframe_counter_, good_matches.size(), ransac_inliers.size(), keyframe_threshold);
      
      // 🚨 SE RANSAC FALLISCE, ANALIZZA IL PERCHÉ
      if (ransac_inliers.empty()) {
        RCLCPP_ERROR(this->get_logger(), "🔬❌ RANSAC FAILED! Starting detailed analysis...");
        
        // Analisi dettagliata dei fallimenti
        analyzeRansacFailures(good_matches, undistorted_prev_keypoints, undistorted_keypoints, 
                             prev_pose_, T, keyframe_threshold);
        
        // Test threshold progressivi
        testProgressiveThresholds(good_matches, undistorted_prev_keypoints, undistorted_keypoints, 
                                 prev_pose_, T);
                                 
        RCLCPP_ERROR(this->get_logger(), "🔬🔍 RANSAC analysis complete. Check logs above for details.");
      }
      
      if (!ransac_inliers.empty()) {
        // Triangolazione con keypoints corretti per distorsione
        std::vector<pcl::PointXYZ> new_points;
        for (const auto& match : ransac_inliers) {
          // USA KEYPOINTS NON-DISTORTI per triangolazione!
          pcl::PointXYZ point = triangulatePoint(undistorted_prev_keypoints[match.queryIdx],
                                                 undistorted_keypoints[match.trainIdx],
                                                 prev_pose_, T);
          
          // Controlli di validità per endoscopi
          Eigen::Vector3d p3d(point.x, point.y, point.z);
          double depth = p3d.norm();
          
          if (depth > 0.01 && depth < 10.0) { // 1cm a 10m per endoscopi
            new_points.push_back(point);
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ 🔬 Keyframe %d: Triangulated %zu valid points", 
                    keyframe_counter_, new_points.size());
        
        if (!new_points.empty()) {
          // Usa keypoints originali per la mappa
          addPointsToMap(new_points, ransac_inliers, keypoints);
          RCLCPP_INFO(this->get_logger(), "🗺️ 🔬 Endoscope map now contains %zu points total", 
                      point_map_.size());
          
          // Pubblica sempre dopo keyframes di successo
          publishAccumulatedPointCloud();
          
          RCLCPP_INFO(this->get_logger(), "🎉 🔬 SUCCESSFUL KEYFRAME PROCESSING! Generated %zu 3D points", new_points.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "🔬 Keyframe %d: No valid 3D points after depth filtering", 
                      keyframe_counter_);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "🔬 Keyframe %d: No inliers found after RANSAC - see analysis above", 
                    keyframe_counter_);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "🔬 Keyframe %d: Not enough good matches (%zu < 12)", 
                  keyframe_counter_, good_matches.size());
      
      // Debug anche quando non ci sono abbastanza matches
      if (good_matches.size() > 0) {
        RCLCPP_WARN(this->get_logger(), "🔬🔍 Limited matches available - running analysis anyway...");
        analyzeRansacFailures(good_matches, undistorted_prev_keypoints, undistorted_keypoints, 
                             prev_pose_, T, keyframe_threshold);
      }
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "🔬 First keyframe or no previous descriptors available");
  }

update_keyframe_data:
  // Salva keypoints ORIGINALI (con distorsione) per consistency
  prev_keypoints_ = keypoints;
  prev_descriptors_ = descriptors.clone();
  
  RCLCPP_DEBUG(this->get_logger(), "🔬 Keyframe %d processing complete", keyframe_counter_);
}

// ===============================================
// FUNZIONI GEOMETRICHE (triangolazione, projection, etc.)
// ===============================================

pcl::PointXYZ SfMNode::triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                         const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2) {
  
  // Normalizza coordinate pixel → coordinate camera normalizzate
  Eigen::Vector3d ray1((kp1.pt.x - cx_) / fx_, (kp1.pt.y - cy_) / fy_, 1.0);
  Eigen::Vector3d ray2((kp2.pt.x - cx_) / fx_, (kp2.pt.y - cy_) / fy_, 1.0);
  
  // ray1 e ray2 sono ora in coordinate camera OpenCV (Z=1 entrante)
  
  // Estrai posizioni e rotazioni delle camere
  Eigen::Vector3d t1 = pose1.block<3, 1>(0, 3);
  Eigen::Vector3d t2 = pose2.block<3, 1>(0, 3);
  Eigen::Matrix3d R1 = pose1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = pose2.block<3, 3>(0, 0);
  
  // Trasforma ray in coordinate mondo
  Eigen::Vector3d ray1_world = R1 * ray1;
  Eigen::Vector3d ray2_world = R2 * ray2;
  
  // 🔧 TRIANGOLAZIONE LINEARE ROBUSTA (DLT)
  // Risolve il sistema: P1 * X = kp1 e P2 * X = kp2
  
  // Costruisci matrici di proiezione P = K * [R|t]
  Eigen::Matrix<double, 3, 4> P1, P2;
  P1.block<3, 3>(0, 0) = R1;
  P1.block<3, 1>(0, 3) = t1;
  P2.block<3, 3>(0, 0) = R2;
  P2.block<3, 1>(0, 3) = t2;
  
  // Sistema AX = 0 per punto omogeneo
  Eigen::Matrix<double, 4, 4> A;
  
  // Prima camera: u1*(P1_row3) - P1_row1 = 0
  A.row(0) = ray1.x() * P1.row(2) - P1.row(0);
  // Prima camera: v1*(P1_row3) - P1_row2 = 0  
  A.row(1) = ray1.y() * P1.row(2) - P1.row(1);
  // Seconda camera: u2*(P2_row3) - P2_row1 = 0
  A.row(2) = ray2.x() * P2.row(2) - P2.row(0);
  // Seconda camera: v2*(P2_row3) - P2_row2 = 0
  A.row(3) = ray2.y() * P2.row(2) - P2.row(1);
  
  // Risolvi con SVD
  Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d point_homogeneous = svd.matrixV().col(3);
  
  // Normalizza punto omogeneo
  if (std::abs(point_homogeneous[3]) < 1e-10) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Triangulation failed - point at infinity");
    return pcl::PointXYZ(0, 0, 0);
  }
  
  Eigen::Vector3d point_3d = point_homogeneous.head<3>() / point_homogeneous[3];
  
  // 🔬 Debug triangolazione ogni tanto
  static int tri_debug = 0;
  if (++tri_debug % 200 == 0) {
    // Verifica che il punto sia davanti a entrambe le camere
    Eigen::Vector3d p1_cam = R1.transpose() * (point_3d - t1);  // In camera 1
    Eigen::Vector3d p2_cam = R2.transpose() * (point_3d - t2);  // In camera 2
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "🔬 Triangulation: point=(%.3f,%.3f,%.3f), z1=%.3f, z2=%.3f %s",
                 point_3d.x(), point_3d.y(), point_3d.z(), 
                 p1_cam.z(), p2_cam.z(),
                 (p1_cam.z() > 0 && p2_cam.z() > 0) ? "✅" : "❌ behind camera");
  }
  
  return pcl::PointXYZ(point_3d.x(), point_3d.y(), point_3d.z());
}

Eigen::Vector3d SfMNode::projectToCamera(const Eigen::Vector4d& point_3d, 
                                          const Eigen::Matrix4d& camera_pose) {
  // Trasforma punto in coordinate camera
  Eigen::Matrix4d T_cam_world = camera_pose.inverse();
  Eigen::Vector4d point_cam = T_cam_world * point_3d;
  
  // Controlla validità
  if (std::abs(point_cam.z()) < 1e-3) {
    return Eigen::Vector3d(-1, -1, -1); // Valore invalido
  }
  
  // ⚠️  IMPORTANTE: Con la conversione ROS→OpenCV, Z è ora positivo davanti
  if (point_cam.z() <= 0) {
    return Eigen::Vector3d(-1, -1, -1); // Punto dietro la camera
  }
  
  // Proiezione prospettica
  double x_normalized = point_cam.x() / point_cam.z();
  double y_normalized = point_cam.y() / point_cam.z();
  
  // Applica parametri intrinseci
  double u = fx_ * x_normalized + cx_;
  double v = fy_ * y_normalized + cy_;
  
  return Eigen::Vector3d(u, v, point_cam.z());
}


std::vector<cv::DMatch> SfMNode::applyRANSAC(const std::vector<cv::DMatch>& matches,
                                              const std::vector<cv::KeyPoint>& prev_kpts,
                                              const std::vector<cv::KeyPoint>& curr_kpts,
                                              const Eigen::Matrix4d& prev_pose,
                                              const Eigen::Matrix4d& curr_pose) {
  if (matches.size() < static_cast<size_t>(ransac_min_inliers_)) {
    return matches;
  }
  
  std::vector<cv::DMatch> best_inliers;
  int best_inlier_count = 0;
  
  int iterations = std::min(ransac_max_iterations_, 
                           static_cast<int>(matches.size() * matches.size()));
  
  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<cv::DMatch> current_inliers;
    for (const auto& match : matches) {
      if (isInlier(match, prev_kpts, curr_kpts, prev_pose, curr_pose, ransac_threshold_)) {
        current_inliers.push_back(match);
      }
    }
    
    if (static_cast<int>(current_inliers.size()) > best_inlier_count) {
      best_inlier_count = static_cast<int>(current_inliers.size());
      best_inliers = current_inliers;
      
      if (best_inlier_count > static_cast<int>(matches.size() * 0.8)) {
        break;
      }
    }
  }
  
  return best_inliers.size() >= static_cast<size_t>(ransac_min_inliers_) ? 
         best_inliers : std::vector<cv::DMatch>();
}

// ===============================================
// PREPROCESSING E GESTIONE PUNTI 3D
// ===============================================



cv::Mat SfMNode::preprocessImage(const cv::Mat& raw_image) {
    cv::Mat gray_image;
    if (raw_image.channels() == 3) {
        cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = raw_image.clone();
    }
    
    cv::Mat denoised;
    cv::bilateralFilter(gray_image, denoised, 9, 75, 75);
    
    cv::Mat illumination_corrected = correctIllumination(denoised);
    cv::Mat contrast_enhanced = adaptiveContrastEnhancement(illumination_corrected);
    cv::Mat reflection_reduced = reduceSpecularReflections(contrast_enhanced, raw_image);
    cv::Mat sharpened = applyUnsharpMask(reflection_reduced, 1.5, 1.0);
    
    double blur_score = detectMotionBlur(sharpened);
    if (blur_score > 0.7) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "High motion blur detected (%.2f). Consider slower movement.", blur_score);
    }
    
    return sharpened;
}

cv::Mat SfMNode::correctIllumination(const cv::Mat& image) {
    cv::Mat background;
    cv::GaussianBlur(image, background, cv::Size(51, 51), 0);
    
    cv::Mat corrected;
    cv::subtract(image, background, corrected);
    cv::add(corrected, cv::Scalar(128), corrected);
    
    return corrected;
}

cv::Mat SfMNode::adaptiveContrastEnhancement(const cv::Mat& image) {
    cv::Mat enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->setTilesGridSize(cv::Size(8, 8));
    clahe->apply(image, enhanced);
    
    cv::Mat blended;
    cv::addWeighted(image, 0.7, enhanced, 0.3, 0, blended);
    return blended;
}

cv::Mat SfMNode::reduceSpecularReflections(const cv::Mat& gray_image, const cv::Mat& color_image) {
    cv::Mat lab_image;
    cv::cvtColor(color_image, lab_image, cv::COLOR_BGR2Lab);
    
    std::vector<cv::Mat> lab_channels;
    cv::split(lab_image, lab_channels);
    cv::Mat lightness = lab_channels[0];
    
    cv::Mat specular_mask;
    cv::threshold(lightness, specular_mask, 240, 255, cv::THRESH_BINARY);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(specular_mask, specular_mask, kernel);
    
    cv::Mat result;
    cv::inpaint(gray_image, specular_mask, result, 3, cv::INPAINT_TELEA);
    return result;
}

cv::Mat SfMNode::applyUnsharpMask(const cv::Mat& image, double strength, double threshold) {
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(0, 0), 1.0);
    
    cv::Mat high_pass;
    cv::subtract(image, blurred, high_pass);
    
    cv::Mat mask;
    cv::threshold(cv::abs(high_pass), mask, threshold, 1.0, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8UC1);
    
    cv::Mat sharpened;
    cv::addWeighted(image, 1.0, high_pass, strength, 0, sharpened, CV_8UC1);
    return sharpened;
}

double SfMNode::detectMotionBlur(const cv::Mat& image) {
    cv::Mat laplacian;
    cv::Laplacian(image, laplacian, CV_64F);
    
    cv::Scalar mu, sigma;
    cv::meanStdDev(laplacian, mu, sigma);
    
    double variance = sigma.val[0] * sigma.val[0];
    double blur_score = 1.0 - std::clamp(variance / 1000.0, 0.0, 1.0);
    return blur_score;
}

void SfMNode::logPreprocessingStats(const cv::Mat& original, const cv::Mat& processed) {
    cv::Scalar original_mean = cv::mean(original);
    cv::Scalar processed_mean = cv::mean(processed);
    
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {
        RCLCPP_DEBUG(this->get_logger(), 
                    "Preprocessing: Mean %.1f->%.1f", 
                    original_mean.val[0], processed_mean.val[0]);
    }
}

// ===============================================
// GESTIONE POINT CLOUD 
// ===============================================

void SfMNode::addPointsToMap(const std::vector<pcl::PointXYZ>& new_points,
                             const std::vector<cv::DMatch>& matches,
                             const std::vector<cv::KeyPoint>& current_keypoints) {
  
  if (!accumulated_cloud_->empty()) {
    kdtree_.setInputCloud(accumulated_cloud_);
  }
  
  for (size_t i = 0; i < new_points.size(); ++i) {
    const auto& point = new_points[i];
    bool is_duplicate = false;
    
    if (!accumulated_cloud_->empty()) {
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      
      if (kdtree_.radiusSearch(point, min_point_distance_, 
                               neighbor_indices, neighbor_distances) > 0) {
        int closest_idx = neighbor_indices[0];
        if (closest_idx < static_cast<int>(point_map_.size())) {
          point_map_[static_cast<size_t>(closest_idx)].observation_count++;
          point_map_[static_cast<size_t>(closest_idx)].frame_ids.push_back(keyframe_counter_);
          point_map_[static_cast<size_t>(closest_idx)].observations.push_back(current_keypoints[matches[i].trainIdx]);
          is_duplicate = true;
        }
      }
    }
    
    if (!is_duplicate) {
      PointWithObservations new_point_obs;
      new_point_obs.point = point;
      new_point_obs.frame_ids.push_back(keyframe_counter_);
      new_point_obs.observations.push_back(current_keypoints[matches[i].trainIdx]);
      new_point_obs.observation_count = 1;
      new_point_obs.quality_score = calculatePointQuality(point, {current_keypoints[matches[i].trainIdx]});
      
      point_map_.push_back(new_point_obs);
      accumulated_cloud_->push_back(point);
    }
  }
  
  if (point_map_.size() > static_cast<size_t>(max_points_)) {
    filterPointCloud();
  }
}

double SfMNode::calculatePointQuality(const pcl::PointXYZ& point, 
                                       const std::vector<cv::KeyPoint>& observations) {
  double quality = 0.0;
  quality += observations.size() * 0.5;
  
  double avg_response = 0.0;
  for (const auto& kp : observations) {
    avg_response += kp.response;
  }
  quality += (avg_response / observations.size()) * 0.3;
  
  double depth = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
  if (depth > 1.0 && depth < 10.0) {
    quality += 0.2;
  }
  
  return quality;
}

void SfMNode::filterPointCloud() {
  std::sort(point_map_.begin(), point_map_.end(),
            [](const PointWithObservations& a, const PointWithObservations& b) {
              return a.quality_score > b.quality_score;
            });
  
  auto it = std::remove_if(point_map_.begin(), point_map_.end(),
                           [this](const PointWithObservations& p) {
                             return p.observation_count < min_observations_;
                           });
  point_map_.erase(it, point_map_.end());
  
  if (point_map_.size() > static_cast<size_t>(max_points_)) {
    point_map_.resize(static_cast<size_t>(max_points_));
  }
  
  accumulated_cloud_->clear();
  for (const auto& point_obs : point_map_) {
    accumulated_cloud_->push_back(point_obs.point);
  }
}

void SfMNode::publishAccumulatedPointCloud() {
  if (accumulated_cloud_->empty()) return;
  
  accumulated_cloud_->width = accumulated_cloud_->points.size();
  accumulated_cloud_->height = 1;
  accumulated_cloud_->is_dense = true;
  
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*accumulated_cloud_, cloud_msg);
  cloud_msg.header.frame_id = world_frame_;
  cloud_msg.header.stamp = this->now();
  pc_pub_->publish(cloud_msg);
  
  // Filtered cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(accumulated_cloud_);
  voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_filter.filter(*filtered_cloud);
  
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
  outlier_filter.setInputCloud(filtered_cloud);
  outlier_filter.setRadiusSearch(outlier_radius_);
  outlier_filter.setMinNeighborsInRadius(outlier_min_neighbors_);
  outlier_filter.filter(*filtered_cloud);
  
  if (!filtered_cloud->empty()) {
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header.frame_id = world_frame_;
    filtered_msg.header.stamp = this->now();
    filtered_pc_pub_->publish(filtered_msg);
    
    RCLCPP_INFO(this->get_logger(), "🔬 Published clouds: raw=%zu, filtered=%zu points",
                accumulated_cloud_->size(), filtered_cloud->size());
  }
}

// ===============================================
// MAIN
// ===============================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SfMNode>());
  rclcpp::shutdown();
  return 0;
}

// ===============================================
// FUNZIONI DEBUG COMPLETE - AGGIUNGI AL TUO CPP
// ===============================================

void SfMNode::debugCameraPoses(const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2, int frame_num) {
  // Estrai traslazioni e rotazioni
  Eigen::Vector3d t1 = pose1.block<3,1>(0,3);
  Eigen::Vector3d t2 = pose2.block<3,1>(0,3);
  Eigen::Matrix3d R1 = pose1.block<3,3>(0,0);
  Eigen::Matrix3d R2 = pose2.block<3,3>(0,0);
  
  // Calcola baseline
  double baseline = (t2 - t1).norm();
  
  // Calcola differenza angolare
  Eigen::Matrix3d dR = R1.transpose() * R2;
  Eigen::AngleAxisd angle_diff(dR);
  double angle_degrees = angle_diff.angle() * 180.0 / M_PI;
  
  RCLCPP_INFO(this->get_logger(), 
              "🔬🎯 POSE DEBUG Frame %d:\n"
              "   📍 Pose1 translation: [%.6f, %.6f, %.6f]\n"
              "   📍 Pose2 translation: [%.6f, %.6f, %.6f]\n"
              "   📏 Baseline: %.6f meters\n"
              "   🔄 Angular difference: %.2f degrees\n"
              "   ⚖️  Baseline sufficient (>%.3fm): %s\n"
              "   ⚖️  Rotation sufficient (>%.1f°): %s",
              frame_num,
              t1.x(), t1.y(), t1.z(),
              t2.x(), t2.y(), t2.z(),
              baseline, angle_degrees,
              min_baseline_meters_, baseline > min_baseline_meters_ ? "✅" : "❌",
              min_rotation_degrees_, angle_degrees > min_rotation_degrees_ ? "✅" : "❌");
  
  // Verifica se le pose sono troppo simili
  if (baseline < 0.001 && angle_degrees < 0.1) {
    RCLCPP_ERROR(this->get_logger(), 
                 "🚨 CRITICAL: Camera poses are nearly identical!\n"
                 "   This will cause triangulation to fail!\n"
                 "   Check your TF publishing system.");
  }
  
  // Debug delle matrici di rotazione ogni 5 keyframe
  static int matrix_debug_counter = 0;
  if (++matrix_debug_counter % 5 == 0) {
    RCLCPP_DEBUG(this->get_logger(), "🔬 Rotation Matrix 1:");
    for(int i = 0; i < 3; i++) {
      RCLCPP_DEBUG(this->get_logger(), "   [%.6f, %.6f, %.6f]", R1(i,0), R1(i,1), R1(i,2));
    }
    RCLCPP_DEBUG(this->get_logger(), "🔬 Rotation Matrix 2:");
    for(int i = 0; i < 3; i++) {
      RCLCPP_DEBUG(this->get_logger(), "   [%.6f, %.6f, %.6f]", R2(i,0), R2(i,1), R2(i,2));
    }
  }
}

void SfMNode::analyzeRansacFailures(const std::vector<cv::DMatch>& good_matches,
                                   const std::vector<cv::KeyPoint>& undistorted_prev_keypoints,
                                   const std::vector<cv::KeyPoint>& undistorted_keypoints,
                                   const Eigen::Matrix4d& prev_pose,
                                   const Eigen::Matrix4d& curr_pose,
                                   double threshold) {
  
  struct RansacStats {
    int total_matches = 0;
    int invalid_indices = 0;
    int triangulation_failed = 0;
    int invalid_depth = 0;
    int behind_camera = 0;
    int invalid_projection = 0;
    int high_reprojection_error = 0;
    int successful_inliers = 0;
  } stats;
  
  std::vector<double> all_errors_1, all_errors_2;
  
  for (const auto& match : good_matches) {
    stats.total_matches++;
    
    // Test 1: Controllo indici
    if (static_cast<size_t>(match.queryIdx) >= undistorted_prev_keypoints.size() || 
        static_cast<size_t>(match.trainIdx) >= undistorted_keypoints.size()) {
      stats.invalid_indices++;
      continue;
    }
    
    const cv::KeyPoint& kp1 = undistorted_prev_keypoints[match.queryIdx];
    const cv::KeyPoint& kp2 = undistorted_keypoints[match.trainIdx];
    
    // Test 2: Triangolazione
    pcl::PointXYZ point_3d = triangulatePoint(kp1, kp2, prev_pose, curr_pose);
    if (std::abs(point_3d.x) < 1e-6 && std::abs(point_3d.y) < 1e-6 && std::abs(point_3d.z) < 1e-6) {
      stats.triangulation_failed++;
      continue;
    }
    
    // Test 3: Depth check
    double depth = std::sqrt(point_3d.x*point_3d.x + point_3d.y*point_3d.y + point_3d.z*point_3d.z);
    if (depth < 0.005 || depth > 10.0) {
      stats.invalid_depth++;
      continue;
    }
    
    // Test 4: Camera position check
    Eigen::Vector4d point_homogeneous(point_3d.x, point_3d.y, point_3d.z, 1.0);
    Eigen::Matrix4d T_cam1_world = prev_pose.inverse();
    Eigen::Matrix4d T_cam2_world = curr_pose.inverse();
    Eigen::Vector4d point_cam1 = T_cam1_world * point_homogeneous;
    Eigen::Vector4d point_cam2 = T_cam2_world * point_homogeneous;
    
    if (point_cam1.z() <= 0.005 || point_cam2.z() <= 0.005) {
      stats.behind_camera++;
      continue;
    }
    
    // Test 5: Projection check
    Eigen::Vector3d projected_1 = projectToCamera(point_homogeneous, prev_pose);
    Eigen::Vector3d projected_2 = projectToCamera(point_homogeneous, curr_pose);
    
    if ((projected_1.x() < 0 && projected_1.y() < 0) || 
        (projected_2.x() < 0 && projected_2.y() < 0)) {
      stats.invalid_projection++;
      continue;
    }
    
    // Test 6: Reprojection error
    double error_1 = std::sqrt(std::pow(projected_1.x() - kp1.pt.x, 2) + 
                               std::pow(projected_1.y() - kp1.pt.y, 2));
    double error_2 = std::sqrt(std::pow(projected_2.x() - kp2.pt.x, 2) + 
                               std::pow(projected_2.y() - kp2.pt.y, 2));
    
    all_errors_1.push_back(error_1);
    all_errors_2.push_back(error_2);
    
    if (error_1 < threshold && error_2 < threshold) {
      stats.successful_inliers++;
    } else {
      stats.high_reprojection_error++;
    }
  }
  
  // Report statistiche dettagliate
  RCLCPP_ERROR(this->get_logger(), 
              "🔬🚨 RANSAC FAILURE ANALYSIS:\n"
              "   📊 Total matches processed: %d\n"
              "   ❌ Invalid indices: %d (%.1f%%)\n"
              "   ❌ Triangulation failed: %d (%.1f%%)\n"
              "   ❌ Invalid depth: %d (%.1f%%)\n"
              "   ❌ Behind camera: %d (%.1f%%)\n"
              "   ❌ Invalid projection: %d (%.1f%%)\n"
              "   ❌ High reprojection error: %d (%.1f%%)\n"
              "   ✅ Successful inliers: %d (%.1f%%)", 
              stats.total_matches,
              stats.invalid_indices, 100.0*stats.invalid_indices/stats.total_matches,
              stats.triangulation_failed, 100.0*stats.triangulation_failed/stats.total_matches,
              stats.invalid_depth, 100.0*stats.invalid_depth/stats.total_matches,
              stats.behind_camera, 100.0*stats.behind_camera/stats.total_matches,
              stats.invalid_projection, 100.0*stats.invalid_projection/stats.total_matches,
              stats.high_reprojection_error, 100.0*stats.high_reprojection_error/stats.total_matches,
              stats.successful_inliers, 100.0*stats.successful_inliers/stats.total_matches);
  
  // Analisi errori di reproiezione
  if (!all_errors_1.empty()) {
    std::sort(all_errors_1.begin(), all_errors_1.end());
    std::sort(all_errors_2.begin(), all_errors_2.end());
    
    double median_error_1 = all_errors_1[all_errors_1.size()/2];
    double median_error_2 = all_errors_2[all_errors_2.size()/2];
    double max_error_1 = all_errors_1.back();
    double max_error_2 = all_errors_2.back();
    double min_error_1 = all_errors_1.front();
    double min_error_2 = all_errors_2.front();
    
    RCLCPP_ERROR(this->get_logger(), 
                "🔬📊 REPROJECTION ERROR ANALYSIS:\n"
                "   📊 Camera 1 errors: min=%.2f, median=%.2f, max=%.2f pixels\n"
                "   📊 Camera 2 errors: min=%.2f, median=%.2f, max=%.2f pixels\n"
                "   🎯 Current threshold: %.2f pixels\n"
                "   💡 Suggested threshold: %.2f pixels (median * 3)", 
                min_error_1, median_error_1, max_error_1,
                min_error_2, median_error_2, max_error_2,
                threshold, std::max(median_error_1, median_error_2) * 3.0);
  }
}

void SfMNode::testProgressiveThresholds(const std::vector<cv::DMatch>& good_matches,
                                       const std::vector<cv::KeyPoint>& undistorted_prev_keypoints,
                                       const std::vector<cv::KeyPoint>& undistorted_keypoints,
                                       const Eigen::Matrix4d& prev_pose,
                                       const Eigen::Matrix4d& curr_pose) {
  
  std::vector<double> test_thresholds = {100.0, 50.0, 20.0, 10.0, 5.0, 3.0, 2.0, 1.0};
  
  RCLCPP_WARN(this->get_logger(), "🔬🧪 PROGRESSIVE THRESHOLD TEST:");
  
  bool found_working_threshold = false;
  
  for (double test_thresh : test_thresholds) {
    int inlier_count = 0;
    std::vector<double> errors_1, errors_2;
    
    for (const auto& match : good_matches) {
      if (static_cast<size_t>(match.queryIdx) >= undistorted_prev_keypoints.size() || 
          static_cast<size_t>(match.trainIdx) >= undistorted_keypoints.size()) {
        continue;
      }
      
      const cv::KeyPoint& kp1 = undistorted_prev_keypoints[match.queryIdx];
      const cv::KeyPoint& kp2 = undistorted_keypoints[match.trainIdx];
      
      // Test completo come isInlierUndistorted ma semplificato
      pcl::PointXYZ point_3d = triangulatePoint(kp1, kp2, prev_pose, curr_pose);
      
      if (std::abs(point_3d.x) < 1e-6 && std::abs(point_3d.y) < 1e-6 && std::abs(point_3d.z) < 1e-6) {
        continue;
      }
      
      double depth = std::sqrt(point_3d.x*point_3d.x + point_3d.y*point_3d.y + point_3d.z*point_3d.z);
      if (depth < 0.005 || depth > 10.0) {
        continue;
      }
      
      Eigen::Vector4d point_homogeneous(point_3d.x, point_3d.y, point_3d.z, 1.0);
      Eigen::Matrix4d T_cam1_world = prev_pose.inverse();
      Eigen::Matrix4d T_cam2_world = curr_pose.inverse();
      Eigen::Vector4d point_cam1 = T_cam1_world * point_homogeneous;
      Eigen::Vector4d point_cam2 = T_cam2_world * point_homogeneous;
      
      if (point_cam1.z() <= 0.005 || point_cam2.z() <= 0.005) {
        continue;
      }
      
      Eigen::Vector3d projected_1 = projectToCamera(point_homogeneous, prev_pose);
      Eigen::Vector3d projected_2 = projectToCamera(point_homogeneous, curr_pose);
      
      if ((projected_1.x() < 0 && projected_1.y() < 0) || 
          (projected_2.x() < 0 && projected_2.y() < 0)) {
        continue;
      }
      
      double error_1 = std::sqrt(std::pow(projected_1.x() - kp1.pt.x, 2) + 
                                 std::pow(projected_1.y() - kp1.pt.y, 2));
      double error_2 = std::sqrt(std::pow(projected_2.x() - kp2.pt.x, 2) + 
                                 std::pow(projected_2.y() - kp2.pt.y, 2));
      
      errors_1.push_back(error_1);
      errors_2.push_back(error_2);
      
      if (error_1 < test_thresh && error_2 < test_thresh) {
        inlier_count++;
      }
    }
    
    double percentage = (good_matches.empty()) ? 0.0 : (100.0 * inlier_count / good_matches.size());
    
    std::string status = "";
    if (inlier_count >= 8) {
      status = "✅ SUFFICIENT";
      if (!found_working_threshold) {
        found_working_threshold = true;
        status += " ⭐ FIRST SUCCESS";
      }
    } else {
      status = "❌ NOT ENOUGH";
    }
    
    RCLCPP_WARN(this->get_logger(), 
                "   🎯 Threshold %5.1f px: %3d/%3zu inliers (%5.1f%%) %s", 
                test_thresh, inlier_count, good_matches.size(), percentage, status.c_str());
    
    // Se troviamo il primo threshold funzionante, mostra dettagli
    if (inlier_count >= 8 && !errors_1.empty() && found_working_threshold && test_thresh <= 20.0) {
      std::sort(errors_1.begin(), errors_1.end());
      std::sort(errors_2.begin(), errors_2.end());
      
      RCLCPP_WARN(this->get_logger(), 
                  "      📊 Error range: cam1[%.1f-%.1f], cam2[%.1f-%.1f] pixels", 
                  errors_1[0], errors_1.back(), errors_2[0], errors_2.back());
      
      RCLCPP_ERROR(this->get_logger(), 
                   "💡 RECOMMENDATION: Change ransac_threshold to %.1f pixels in your parameters!", 
                   test_thresh);
    }
  }
  
  if (!found_working_threshold) {
    RCLCPP_ERROR(this->get_logger(), 
                 "🚨 CRITICAL: No working threshold found! This suggests:\n"
                 "   1. Camera poses are too similar (baseline too small)\n"
                 "   2. Camera calibration is wrong\n"
                 "   3. Feature matches are fundamentally bad");
  }
}