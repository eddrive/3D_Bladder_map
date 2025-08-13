#include "online_sfm/sfm_node.hpp"
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>

// ===============================================
// COSTRUTTORE CORRETTO
// ===============================================

SfMNode::SfMNode() : Node("endoscope_sfm") {
  // Sottoscrizioni
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/endoscope/image_raw", 10,
    std::bind(&SfMNode::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/endoscope/camera_info", 10,
    std::bind(&SfMNode::cameraInfoCallback, this, std::placeholders::_1));

  // Publisher point cloud
  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sfm/point_cloud", 10);

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Point cloud COLORATA - CORRETTO!
  accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  // PARAMETRI ESSENZIALI PER OGGETTO 20CM
  min_baseline_meters_ = 0.003;      // 3mm - per oggetto piccolo (20cm)
  min_rotation_degrees_ = 0.8;       // Rotazione minima in gradi
  ransac_threshold_ = 12.0;          // Soglia RANSAC per fisheye
  
  RCLCPP_INFO(this->get_logger(), "🔬 Minimal Endoscope SfM ready for 20cm objects (baseline: %.1fmm, rotation: %.1f°)", 
              min_baseline_meters_ * 1000, min_rotation_degrees_);
}

// ===============================================
// CALIBRAZIONE CAMERA
// ===============================================

void SfMNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (cam_info_received_) return;
  
  camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat(msg->d).clone();
  
  fx_ = camera_matrix_.at<double>(0, 0);
  fy_ = camera_matrix_.at<double>(1, 1);
  cx_ = camera_matrix_.at<double>(0, 2);
  cy_ = camera_matrix_.at<double>(1, 2);
  
  // Rileva fisheye
  is_fisheye_ = (msg->distortion_model == "fisheye");
  
  cam_info_received_ = true;
  
  RCLCPP_INFO(this->get_logger(), "📷 Camera %s: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
              is_fisheye_ ? "FISHEYE" : "STANDARD", fx_, fy_, cx_, cy_);
}

// ===============================================
// POSE CAMERA - CORRETTO TF
// ===============================================

bool SfMNode::getCameraPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_world_cam) {
  try {
    auto tf_msg = tf_buffer_->lookupTransform("world", "camera", stamp, 
                                              rclcpp::Duration::from_nanoseconds(100000000));
    
    Eigen::Translation3d t(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);
    Eigen::Quaterniond q(tf_msg.transform.rotation.w,
                         tf_msg.transform.rotation.x,
                         tf_msg.transform.rotation.y,
                         tf_msg.transform.rotation.z);
    
    T_world_cam = (t * q).matrix();
    
    // Debug ogni 20 frame
    static int debug_count = 0;
    if (++debug_count % 20 == 0) {
      Eigen::Vector3d pos = T_world_cam.block<3,1>(0,3);
      Eigen::Vector3d z_axis = T_world_cam.block<3,3>(0,0).col(2);
      
      RCLCPP_INFO(this->get_logger(), 
                  "📷 Camera: pos(%.3f,%.3f,%.3f), Z→(%.3f,%.3f,%.3f)", 
                  pos.x(), pos.y(), pos.z(),
                  z_axis.x(), z_axis.y(), z_axis.z());
    }
    
    return true;
    
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                          "No camera pose: %s", e.what());
    return false;
  }
}
// ===============================================
// CALLBACK IMMAGINE
// ===============================================

void SfMNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!cam_info_received_) return;
  
  // Ottieni pose camera
  Eigen::Matrix4d current_pose;
  if (!getCameraPose(msg->header.stamp, current_pose)) return;
  
  // Controlla se processare questo frame (movimento O rotazione)
  if (has_prev_pose_) {
    Eigen::Vector3d t_prev = prev_pose_.block<3,1>(0,3);
    Eigen::Vector3d t_curr = current_pose.block<3,1>(0,3);
    double baseline = (t_curr - t_prev).norm();
    
    // Calcola anche differenza angolare
    Eigen::Matrix3d R_prev = prev_pose_.block<3,3>(0,0);
    Eigen::Matrix3d R_curr = current_pose.block<3,3>(0,0);
    Eigen::Matrix3d dR = R_prev.transpose() * R_curr;
    Eigen::AngleAxisd angle_diff(dR);
    double angle_degrees = std::abs(angle_diff.angle() * 180.0 / M_PI);
    
    bool baseline_ok = baseline > min_baseline_meters_;
    bool rotation_ok = angle_degrees > min_rotation_degrees_;
    
    if (!baseline_ok && !rotation_ok) {
      return; // Non abbastanza movimento
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Keyframe trigger: baseline=%.1fmm, rotation=%.1f°", 
                baseline*1000, angle_degrees);
  }
  
  // Converti immagine
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "🔬 Processing keyframe %d", keyframe_count_);
  
  // PROCESSA IL FRAME
  processFrame(image, current_pose);
  
  // Salva per il prossimo frame
  prev_pose_ = current_pose;
  has_prev_pose_ = true;
  keyframe_count_++;
}

// ===============================================
// PROCESSAMENTO FRAME - COMPLETO
// ===============================================

void SfMNode::processFrame(const cv::Mat& image, const Eigen::Matrix4d& pose) {
  // 🔍 DIAGNOSTICA 1: Analizza immagine RAW (prima di qualsiasi elaborazione)
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  
  cv::Scalar mean_raw, stddev_raw;
  cv::meanStdDev(gray, mean_raw, stddev_raw);
  double raw_contrast = stddev_raw[0];
  double raw_brightness = mean_raw[0];
  
  RCLCPP_INFO(this->get_logger(), 
              "🔍 RAW IMAGE ANALYSIS:\n"
              "   📊 Size: %dx%d\n"
              "   💡 Brightness: %.1f (good range: 50-200)\n"
              "   📈 Contrast: %.1f (good range: >15)\n"
              "   📊 Type: %d, Channels: %d",
              gray.cols, gray.rows, raw_brightness, raw_contrast, 
              gray.type(), image.channels());
  
  // Undistortion come prima
  cv::Mat undistorted_gray, undistorted_color;
  if (is_fisheye_) {
    cv::fisheye::undistortImage(gray, undistorted_gray, camera_matrix_, dist_coeffs_, camera_matrix_);
    cv::fisheye::undistortImage(image, undistorted_color, camera_matrix_, dist_coeffs_, camera_matrix_);
  } else {
    cv::undistort(gray, undistorted_gray, camera_matrix_, dist_coeffs_, camera_matrix_);
    cv::undistort(image, undistorted_color, camera_matrix_, dist_coeffs_, camera_matrix_);
  }
  
  // 🔍 DIAGNOSTICA 2: Analizza immagine UNDISTORTED
  cv::Scalar mean_undist, stddev_undist;
  cv::meanStdDev(undistorted_gray, mean_undist, stddev_undist);
  double undist_contrast = stddev_undist[0];
  double undist_brightness = mean_undist[0];
  
  RCLCPP_INFO(this->get_logger(), 
              "🔍 UNDISTORTED ANALYSIS:\n"
              "   💡 Brightness: %.1f (change: %+.1f)\n"
              "   📈 Contrast: %.1f (change: %+.1f)\n"
              "   📊 Impact: %s",
              undist_brightness, undist_brightness - raw_brightness,
              undist_contrast, undist_contrast - raw_contrast,
              undist_contrast < raw_contrast ? "DEGRADED" : "IMPROVED");
  
  // 🔍 DIAGNOSTICA 3: Test CLAHE enhancement
  cv::Mat enhanced_image;
  bool clahe_applied = false;
  
  if (undist_contrast < 20.0) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Low contrast detected (%.1f < 20), testing CLAHE enhancement", undist_contrast);
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3.0);
    clahe->setTilesGridSize(cv::Size(8, 8));
    clahe->apply(undistorted_gray, enhanced_image);
    
    cv::Scalar mean_enhanced, stddev_enhanced;
    cv::meanStdDev(enhanced_image, mean_enhanced, stddev_enhanced);
    double enhanced_contrast = stddev_enhanced[0];
    double enhanced_brightness = mean_enhanced[0];
    
    RCLCPP_INFO(this->get_logger(), 
                "✨ CLAHE ENHANCEMENT RESULTS:\n"
                "   💡 Brightness: %.1f -> %.1f (change: %+.1f)\n"
                "   📈 Contrast: %.1f -> %.1f (improvement: %.1fx)\n"
                "   📊 Quality: %s",
                undist_brightness, enhanced_brightness, enhanced_brightness - undist_brightness,
                undist_contrast, enhanced_contrast, enhanced_contrast / undist_contrast,
                enhanced_contrast > 15.0 ? "GOOD" : "STILL_POOR");
    
    clahe_applied = true;
  } else {
    enhanced_image = undistorted_gray.clone();
  }
  
  // 🔍 DIAGNOSTICA 4: Analisi histogram per capire distribuzione pixel
  std::vector<cv::Mat> hist_images = {gray, undistorted_gray};
  std::vector<std::string> hist_names = {"RAW", "UNDISTORTED"};
  
  if (clahe_applied) {
    hist_images.push_back(enhanced_image);
    hist_names.push_back("ENHANCED");
  }
  
  for (size_t i = 0; i < hist_images.size(); ++i) {
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::calcHist(&hist_images[i], 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    
    // Trova picchi e distribuzione
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(hist, &minVal, &maxVal, &minLoc, &maxLoc);
    
    // Calcola percentili
    float total_pixels = hist_images[i].rows * hist_images[i].cols;
    float cumulative = 0;
    int p5 = 0, p95 = 255;
    
    for (int j = 0; j < 256; ++j) {
      cumulative += hist.at<float>(j);
      if (cumulative >= total_pixels * 0.05 && p5 == 0) p5 = j;
      if (cumulative >= total_pixels * 0.95 && p95 == 255) { p95 = j; break; }
    }
    
    RCLCPP_INFO(this->get_logger(),
                "📊 %s HISTOGRAM:\n"
                "   📈 Peak intensity: %d (count: %.0f)\n"
                "   📊 5th percentile: %d\n"
                "   📊 95th percentile: %d\n"
                "   📏 Dynamic range: %d (good: >100)",
                hist_names[i].c_str(), maxLoc.y, maxVal, p5, p95, p95 - p5);
  }
  
  // Feature detection con parametri ORIGINALI per diagnostica
  auto sift = cv::SIFT::create(2000, 3, 0.04, 10, 1.6);  // Parametri standard
  
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift->detectAndCompute(enhanced_image, cv::noArray(), keypoints, descriptors);
  
  // 🔍 DIAGNOSTICA 5: Analisi dettagliatissima features
  if (!keypoints.empty()) {
    std::vector<float> responses;
    std::vector<float> scales;
    std::vector<cv::Point2f> positions;
    
    for (const auto& kp : keypoints) {
      responses.push_back(kp.response);
      scales.push_back(kp.size);
      positions.push_back(kp.pt);
    }
    
    std::sort(responses.begin(), responses.end(), std::greater<float>());
    std::sort(scales.begin(), scales.end(), std::greater<float>());
    
    float max_response = responses[0];
    float min_response = responses.back();
    float avg_response = std::accumulate(responses.begin(), responses.end(), 0.0f) / responses.size();
    float median_response = responses[responses.size() / 2];
    float top25_response = responses[std::min(responses.size()-1, responses.size()/4)];
    
    // Conta features per range di response
    auto count_range = [&responses](float min, float max) {
      return std::count_if(responses.begin(), responses.end(), 
                          [min, max](float r){ return r >= min && r < max; });
    };
    
    RCLCPP_INFO(this->get_logger(),
                "🔍 DETAILED FEATURE ANALYSIS:\n"
                "   📊 Total features detected: %zu\n"
                "   📈 Response stats:\n"
                "      Max: %.4f\n"
                "      Top 25%%: %.4f\n" 
                "      Median: %.4f\n"
                "      Average: %.4f\n"
                "      Min: %.4f\n"
                "   📊 Response distribution:\n"
                "      >100: %zu features\n"
                "      >10: %zu features\n"
                "      >1.0: %zu features\n"
                "      >0.1: %zu features\n"
                "      >0.01: %zu features\n"
                "      >0.001: %zu features\n"
                "   📏 Scale range: %.1f - %.1f",
                keypoints.size(),
                max_response, top25_response, median_response, avg_response, min_response,
                count_range(100.0f, 10000.0f),
                count_range(10.0f, 100.0f),
                count_range(1.0f, 10.0f),
                count_range(0.1f, 1.0f),
                count_range(0.01f, 0.1f),
                count_range(0.001f, 0.01f),
                scales[0], scales.back());
    
    // 🔍 DIAGNOSTICA 6: Distribuzione spaziale features
    int center_features = 0, border_features = 0;
    int cx_img = enhanced_image.cols / 2;
    int cy_img = enhanced_image.rows / 2;
    int radius = std::min(enhanced_image.cols, enhanced_image.rows) / 4;
    
    for (const auto& pos : positions) {
      double dist_from_center = std::sqrt(std::pow(pos.x - cx_img, 2) + std::pow(pos.y - cy_img, 2));
      if (dist_from_center < radius) {
        center_features++;
      } else {
        border_features++;
      }
    }
    
    RCLCPP_INFO(this->get_logger(),
                "📍 SPATIAL DISTRIBUTION:\n"
                "   🎯 Center features (<%dpx from center): %d\n"
                "   🔲 Border features: %d\n"
                "   📊 Center/Border ratio: %.2f (good: >0.5)",
                radius, center_features, border_features,
                border_features > 0 ? (float)center_features / border_features : 999.0f);
    
    // ===============================================
    // PROCESSO SfM ORIGINALE (con diagnostica aggiunta)
    // ===============================================
    
    RCLCPP_INFO(this->get_logger(), "🔍 Frame %d: %zu features detected", keyframe_count_, keypoints.size());
    
    if (has_prev_frame_ && !prev_descriptors_.empty() && keypoints.size() > 50) {
      
      // Match features
      cv::BFMatcher matcher;
      std::vector<std::vector<cv::DMatch>> knn_matches;
      matcher.knnMatch(prev_descriptors_, descriptors, knn_matches, 2);
      
      // Lowe's ratio test
      std::vector<cv::DMatch> good_matches;
      for (const auto& match_pair : knn_matches) {
        if (match_pair.size() == 2 && 
            match_pair[0].distance < 0.8f * match_pair[1].distance) {
          good_matches.push_back(match_pair[0]);
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "🔗 %zu matches found", good_matches.size());
      
      if (good_matches.size() >= 20) {
        
        // Undistort keypoints (NON SERVE se già lavori su immagine undistorted)
        // std::vector<cv::KeyPoint> prev_undist = undistortKeypoints(prev_keypoints_);
        // std::vector<cv::KeyPoint> curr_undist = undistortKeypoints(keypoints);
        
        // Triangola punti CON COLORE (usa keypoints direttamente se immagine già corretta)
        std::vector<pcl::PointXYZRGB> new_points;
        for (const auto& match : good_matches) {
          pcl::PointXYZRGB point = triangulatePoint(prev_keypoints_[match.queryIdx],
                                                    keypoints[match.trainIdx],
                                                    prev_pose_, pose,
                                                    prev_image_undistorted_, undistorted_color);
          
          // Controllo validità OTTIMIZZATO PER OGGETTO 20CM
          double depth = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
          if (depth > 0.002 && depth < 0.5) {  // 2mm - 50cm
            new_points.push_back(point);
          }
        }
        
        if (!new_points.empty()) {
          // Aggiungi alla point cloud
          for (const auto& point : new_points) {
            accumulated_cloud_->push_back(point);
          }
          
          publishPointCloud();
          
          RCLCPP_INFO(this->get_logger(), "✅ Added %zu colored points (total: %zu)", 
                      new_points.size(), accumulated_cloud_->size());
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "⏭️ Skipping triangulation: has_prev=%s, desc_empty=%s, features=%zu", 
                  has_prev_frame_ ? "YES" : "NO",
                  prev_descriptors_.empty() ? "YES" : "NO", 
                  keypoints.size());
    }
    
    // Salva per il prossimo frame (usa enhanced_image se CLAHE applicato)
    prev_keypoints_ = keypoints;
    prev_descriptors_ = descriptors.clone();
    
  } else {
    RCLCPP_ERROR(this->get_logger(), 
                 "❌ NO FEATURES DETECTED!\n"
                 "   🔍 Possible causes:\n"
                 "      - Extremely low contrast (%.1f)\n"
                 "      - Uniform/blurred image\n"
                 "      - Invalid image format\n"
                 "      - Overly aggressive undistortion",
                 undist_contrast);
  }
  
  // 🔍 DIAGNOSTICA 7: Raccomandazioni basate sui risultati
  RCLCPP_WARN(this->get_logger(),
              "💡 DIAGNOSTIC RECOMMENDATIONS:\n"
              "   📈 Contrast quality: %s\n"
              "   💡 Brightness quality: %s\n"
              "   🔍 Feature detection: %s\n"
              "   🎯 Suggested actions:\n%s",
              undist_contrast > 20 ? "GOOD" : undist_contrast > 10 ? "POOR" : "CRITICAL",
              (undist_brightness > 50 && undist_brightness < 200) ? "GOOD" : "SUBOPTIMAL", 
              keypoints.size() > 100 ? "EXCELLENT" : keypoints.size() > 20 ? "ACCEPTABLE" : "POOR",
              keypoints.size() == 0 ? "      - Check camera exposure/lighting\n      - Verify camera calibration\n      - Consider different SIFT parameters" :
              undist_contrast < 10 ? "      - Apply CLAHE enhancement\n      - Improve lighting conditions" :
              keypoints.size() < 50 ? "      - Lower SIFT contrast threshold\n      - Add texture to scene" :
              "      - Current settings should work well");
  
  // Salva stato per il prossimo frame
  prev_image_undistorted_ = undistorted_color.clone();
  has_prev_frame_ = true;
}

// ===============================================
// TRIANGOLAZIONE CON COLORE - SIGNATURE CORRETTA
// ===============================================

pcl::PointXYZRGB SfMNode::triangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                                   const Eigen::Matrix4d& pose1, const Eigen::Matrix4d& pose2,
                                                   const cv::Mat& image1, const cv::Mat& image2) {
  
  pcl::PointXYZRGB invalid_point;
  invalid_point.x = invalid_point.y = invalid_point.z = 0;
  invalid_point.r = invalid_point.g = invalid_point.b = 0;
  
  // 🔧 CONTROLLI RILASSATI
  
  // 1. Baseline minima rilassata
  Eigen::Vector3d t1 = pose1.block<3, 1>(0, 3);
  Eigen::Vector3d t2 = pose2.block<3, 1>(0, 3);
  double baseline = (t2 - t1).norm();
  
  if (baseline < 0.003) {  // Era 0.005 - ora 3mm invece di 5mm
    return invalid_point;
  }
  
  // 2. Keypoint response rilassata - NON controllare più (già filtrato)
  
  // 3. Margine immagine rilassato
  int margin = 20;  // Era 50px - ora 20px
  if (kp1.pt.x < margin || kp1.pt.x > (image1.cols - margin) ||
      kp1.pt.y < margin || kp1.pt.y > (image1.rows - margin) ||
      kp2.pt.x < margin || kp2.pt.x > (image2.cols - margin) ||
      kp2.pt.y < margin || kp2.pt.y > (image2.rows - margin)) {
    return invalid_point;
  }
  
  // 4. Triangolazione (invariata)
  Eigen::Vector3d ray1_cam((kp1.pt.x - cx_) / fx_, (kp1.pt.y - cy_) / fy_, 1.0);
  Eigen::Vector3d ray2_cam((kp2.pt.x - cx_) / fx_, (kp2.pt.y - cy_) / fy_, 1.0);
  
  Eigen::Matrix3d R1 = pose1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = pose2.block<3, 3>(0, 0);
  
  Eigen::Vector3d ray1_world = R1 * ray1_cam;
  Eigen::Vector3d ray2_world = R2 * ray2_cam;
  
  // 5. Angolo raggi rilassato
  double ray_angle = std::acos(std::clamp(ray1_world.dot(ray2_world), -1.0, 1.0));
  double ray_angle_degrees = ray_angle * 180.0 / M_PI;
  
  if (ray_angle_degrees < 2.0 || ray_angle_degrees > 178.0) {  // Era 5-175 - ora più permissivo
    return invalid_point;
  }
  
  // 6. Triangolazione robusta (invariata)
  Eigen::Vector3d w = t1 - t2;
  double a = ray1_world.dot(ray1_world);
  double b = ray1_world.dot(ray2_world);
  double c = ray2_world.dot(ray2_world);
  double d = ray1_world.dot(w);
  double e = ray2_world.dot(w);
  
  double denom = a * c - b * b;
  if (std::abs(denom) < 1e-8) {
    return invalid_point;
  }
  
  double s = (b * e - c * d) / denom;
  double t_param = (a * e - b * d) / denom;
  
  // 7. Depth rilassata
  if (s <= 0.005 || t_param <= 0.005) {  // Era 0.01 - ora 5mm
    return invalid_point;
  }
  
  Eigen::Vector3d point1 = t1 + s * ray1_world;
  Eigen::Vector3d point2 = t2 + t_param * ray2_world;
  Eigen::Vector3d point_3d = 0.5 * (point1 + point2);
  
  // 8. Errore triangolazione rilassato
  double triangulation_error = (point1 - point2).norm();
  if (triangulation_error > 0.02) {  // Era 0.01 - ora 2cm
    return invalid_point;
  }
  
  // 9. Range distanza rilassato
  double distance_cam1 = (point_3d - t1).norm();
  if (distance_cam1 < 0.01 || distance_cam1 > 0.5) {  // Era 0.02-0.3 - ora più ampio
    return invalid_point;
  }
  
  // 10. Reprojection error rilassato
  Eigen::Vector3d point_cam1 = R1.transpose() * (point_3d - t1);
  Eigen::Vector3d point_cam2 = R2.transpose() * (point_3d - t2);
  
  double u1_proj = fx_ * (point_cam1.x() / point_cam1.z()) + cx_;
  double v1_proj = fy_ * (point_cam1.y() / point_cam1.z()) + cy_;
  double u2_proj = fx_ * (point_cam2.x() / point_cam2.z()) + cx_;
  double v2_proj = fy_ * (point_cam2.y() / point_cam2.z()) + cy_;
  
  double error1 = std::sqrt(std::pow(u1_proj - kp1.pt.x, 2) + std::pow(v1_proj - kp1.pt.y, 2));
  double error2 = std::sqrt(std::pow(u2_proj - kp2.pt.x, 2) + std::pow(v2_proj - kp2.pt.y, 2));
  
  if (error1 > 5.0 || error2 > 5.0) {  // Era 3.0 - ora 5 pixel
    return invalid_point;
  }
  
  // 11. Estrazione colore (rilassata)
  pcl::PointXYZRGB colored_point;
  colored_point.x = point_3d.x();
  colored_point.y = point_3d.y();
  colored_point.z = point_3d.z();
  
  // Prova estrazione colore con validazione meno rigorosa
  int x1 = static_cast<int>(std::round(kp1.pt.x));
  int y1 = static_cast<int>(std::round(kp1.pt.y));
  int x2 = static_cast<int>(std::round(kp2.pt.x));
  int y2 = static_cast<int>(std::round(kp2.pt.y));
  
  bool color_ok = false;
  
  if (x2 >= 0 && x2 < image2.cols && y2 >= 0 && y2 < image2.rows && image2.channels() == 3) {
    cv::Vec3b color = image2.at<cv::Vec3b>(y2, x2);
    int brightness = (color[0] + color[1] + color[2]) / 3;
    if (brightness > 20 && brightness < 235) {  // Range più ampio
      colored_point.b = color[0];
      colored_point.g = color[1];
      colored_point.r = color[2];
      color_ok = true;
    }
  }
  
  if (!color_ok && x1 >= 0 && x1 < image1.cols && y1 >= 0 && y1 < image1.rows && image1.channels() == 3) {
    cv::Vec3b color = image1.at<cv::Vec3b>(y1, x1);
    int brightness = (color[0] + color[1] + color[2]) / 3;
    if (brightness > 20 && brightness < 235) {
      colored_point.b = color[0];
      colored_point.g = color[1];
      colored_point.r = color[2];
      color_ok = true;
    }
  }
  
  if (!color_ok) {
    // 🟡 GIALLO FLUO invece di grigio per color extraction fallita
    colored_point.r = 255;  // Giallo fluo = RGB(255,255,0)
    colored_point.g = 255;
    colored_point.b = 0;
  }
  
  // 12. Log success più frequente per vedere che funziona
  static int success_count = 0;
  if (++success_count % 10 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "✅ RELAXED POINT #%d: world(%.3f,%.3f,%.3f), errors(%.1f,%.1f)px", 
                success_count, point_3d.x(), point_3d.y(), point_3d.z(), error1, error2);
  }
  
  return colored_point;
}

// ===============================================
// FUNZIONE COLORE PER PROFONDITÀ
// ===============================================

void SfMNode::colorByDepth(pcl::PointXYZRGB& point, double depth) {
  // Mappa profondità 0.01m-0.5m a colori arcobaleno
  double normalized_depth = std::clamp((depth - 0.01) / (0.5 - 0.01), 0.0, 1.0);
  
  // HSV to RGB per arcobaleno
  double hue = (1.0 - normalized_depth) * 240.0; // Blu (240°) → Rosso (0°)
  double saturation = 1.0;
  double value = 1.0;
  
  double c = value * saturation;
  double x = c * (1.0 - std::abs(std::fmod(hue / 60.0, 2.0) - 1.0));
  double m = value - c;
  
  double r, g, b;
  if (hue >= 0 && hue < 60) {
    r = c; g = x; b = 0;
  } else if (hue >= 60 && hue < 120) {
    r = x; g = c; b = 0;
  } else if (hue >= 120 && hue < 180) {
    r = 0; g = c; b = x;
  } else if (hue >= 180 && hue < 240) {
    r = 0; g = x; b = c;
  } else if (hue >= 240 && hue < 300) {
    r = x; g = 0; b = c;
  } else {
    r = c; g = 0; b = x;
  }
  
  point.r = static_cast<uint8_t>((r + m) * 255);
  point.g = static_cast<uint8_t>((g + m) * 255);
  point.b = static_cast<uint8_t>((b + m) * 255);
}

// ===============================================
// PUBBLICAZIONE POINT CLOUD
// ===============================================

void SfMNode::publishPointCloud() {
  if (accumulated_cloud_->empty()) return;
  
  accumulated_cloud_->width = accumulated_cloud_->points.size();
  accumulated_cloud_->height = 1;
  accumulated_cloud_->is_dense = true;
  
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*accumulated_cloud_, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = this->now();
  
  pc_pub_->publish(cloud_msg);
  
  RCLCPP_DEBUG(this->get_logger(), "📤 Published %zu colored points", accumulated_cloud_->size());
}

// ===============================================
// MAIN
// ===============================================

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SfMNode>());
  rclcpp::shutdown();
  return 0;
}