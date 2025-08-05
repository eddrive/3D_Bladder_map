#!/usr/bin/env python3

import os

import sys
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import message_filters
from typing import Optional, Tuple

# Add MiDaS to path
sys.path.append('/midas')
from midas.model_loader import load_model, default_models
from midas.midas_net import MidasNet
from midas.transforms import Resize, NormalizeImage, PrepareForNet

class MidasFisheyeDepthNode(Node):
    """
    Complete ROS2 node for real-time depth estimation using MiDaS with automatic fisheye correction
    Optimized for RTX 3070 and endoscopic imaging with fisheye lenses
    """

    def __init__(self):
        super().__init__('midas_depth_node')
        
        # Declare parameters
        self.setup_parameters()
        
        # Get parameters
        self.load_parameters()
        
        # Initialize device (RTX 3070 optimization)
        self.setup_device()
        
        # Load MiDaS model
        self.load_model()
        
        # Setup transforms for endoscopic images
        self.setup_transforms()
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Fisheye correction variables
        self.camera_matrix = None
        self.dist_coeffs = None
        self.fisheye_correction_ready = False
        self.undistort_maps = None
        self.image_size = None
        
        # Setup publishers and subscribers
        self.setup_ros_interface()
        
        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = self.get_clock().now()
        
        self.get_logger().info('MiDaS Fisheye Depth Node initialized successfully')

    def setup_parameters(self):
        """Declare all ROS2 parameters"""
        # Model parameters
        self.declare_parameter('model_type', 'dpt_hybrid_384')
        self.declare_parameter('input_resolution', [384, 384])
        self.declare_parameter('use_half_precision', True)
        
        # Fisheye correction parameters
        self.declare_parameter('enable_fisheye_correction', True)
        self.declare_parameter('fisheye_alpha', 0.6)  # Controls FOV after correction
        self.declare_parameter('auto_extract_camera_params', True)
        self.declare_parameter('crop_corrected_image', True)
        self.declare_parameter('crop_ratio', 0.85)
        
        # Processing parameters
        self.declare_parameter('camera_downsample_factor', 1)
        self.declare_parameter('apply_endoscopic_preprocessing', True)
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_grid_size', [8, 8])
        
        # Depth parameters
        self.declare_parameter('depth_scale_factor', 1000.0)
        self.declare_parameter('max_depth', 500.0)  # mm, for tabletop objects
        self.declare_parameter('min_depth', 50.0)   # mm, minimum distance
        
        # Performance parameters
        self.declare_parameter('optimize_transforms', True)
        self.declare_parameter('enable_performance_logging', True)

    def load_parameters(self):
        """Load all parameters from ROS2 parameter server"""
        # Model parameters
        self.model_type = self.get_parameter('model_type').value
        self.input_res = self.get_parameter('input_resolution').value
        self.use_half_precision = self.get_parameter('use_half_precision').value
        
        # Fisheye parameters
        self.enable_fisheye_correction = self.get_parameter('enable_fisheye_correction').value
        self.fisheye_alpha = self.get_parameter('fisheye_alpha').value
        self.auto_extract_params = self.get_parameter('auto_extract_camera_params').value
        self.crop_corrected = self.get_parameter('crop_corrected_image').value
        self.crop_ratio = self.get_parameter('crop_ratio').value
        
        # Processing parameters
        self.downsample_factor = self.get_parameter('camera_downsample_factor').value
        self.endoscopic_preprocessing = self.get_parameter('apply_endoscopic_preprocessing').value
        self.clahe_clip_limit = self.get_parameter('clahe_clip_limit').value
        self.clahe_tile_size = self.get_parameter('clahe_tile_grid_size').value
        
        # Depth parameters
        self.depth_scale = self.get_parameter('depth_scale_factor').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        
        # Performance parameters
        self.optimize_transforms = self.get_parameter('optimize_transforms').value
        self.enable_perf_logging = self.get_parameter('enable_performance_logging').value

    def setup_device(self):
        """Initialize and optimize CUDA device for RTX 3070"""
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        if torch.cuda.is_available():
            gpu_name = torch.cuda.get_device_name()
            self.get_logger().info(f'Using GPU: {gpu_name}')
            
            # RTX 3070 specific optimizations
            torch.backends.cudnn.benchmark = True
            torch.backends.cudnn.deterministic = False
            
            # Enable tensor core usage for RTX 3070
            if self.use_half_precision:
                torch.backends.cudnn.allow_tf32 = True
                torch.backends.cuda.matmul.allow_tf32 = True
                
        else:
            self.get_logger().warn('CUDA not available, using CPU (performance will be limited)')

    def load_model(self):
        """Load and optimize MiDaS model"""
        try:
            self.get_logger().info(f'Loading MiDaS model: {self.model_type}')

            # Load model using torch.hub
            if self.model_type == 'dpt_hybrid_384':
                self.model = torch.hub.load('intel-isl/MiDaS', 'DPT_Hybrid', 
                                          pretrained=True, trust_repo=True)
            elif self.model_type == 'dpt_large_384':
                self.model = torch.hub.load('intel-isl/MiDaS', 'DPT_Large', 
                                          pretrained=True, trust_repo=True)
            elif self.model_type == 'midas_v21_small_256':
                self.model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', 
                                          pretrained=True, trust_repo=True)
            else:
                raise ValueError(f'Unsupported model type: {self.model_type}')

            # Move to device and set to evaluation mode
            self.model.to(self.device)
            self.model.eval()

            # Enable half precision on GPU for RTX 3070
            if self.use_half_precision and self.device.type == 'cuda':
                self.model = self.model.half()
                self.get_logger().info('Using half precision (FP16) for optimal RTX 3070 performance')

            # RTX 3070 specific optimizations (without JIT)
            if self.device.type == 'cuda':
                torch.backends.cudnn.benchmark = True
                torch.backends.cudnn.deterministic = False
                if self.use_half_precision:
                    torch.backends.cudnn.allow_tf32 = True
                    torch.backends.cuda.matmul.allow_tf32 = True

            self.get_logger().info(f'MiDaS model loaded successfully on {self.device}')

        except Exception as e:
            self.get_logger().error(f'Failed to load MiDaS model: {str(e)}')
            raise

    def setup_transforms(self):
        """Setup image transforms optimized for fisheye endoscopic images"""
        # Standard MiDaS transforms
        self.midas_transform = transforms.Compose([
            Resize(
                self.input_res[0], self.input_res[1],
                resize_target=None,
                keep_aspect_ratio=True,
                ensure_multiple_of=32,
                resize_method="lower_bound",
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            PrepareForNet(),
        ])

        # CLAHE for endoscopic illumination enhancement
        if self.endoscopic_preprocessing:
            self.clahe = cv2.createCLAHE(
                clipLimit=self.clahe_clip_limit, 
                tileGridSize=tuple(self.clahe_tile_size)
            )

    def setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers with synchronized callbacks"""
        # Subscribers with message filtering for synchronization
        self.image_sub = message_filters.Subscriber(
            self, Image, '/endoscope/image_raw'
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/endoscope/camera_info'
        )

        # Synchronize image and camera info with tight timing
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], 
            queue_size=10,
            slop=0.05  # 50ms tolerance
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publishers for depth data (compatible with RTAB-Map)
        self.depth_pub = self.create_publisher(
            Image, '/endoscope/depth/image_raw', 10
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/endoscope/depth/camera_info', 10
        )

        # Optional: publish corrected RGB image for visualization
        self.corrected_image_pub = self.create_publisher(
            Image, '/endoscope/image_corrected', 10
        )

        # Performance monitoring timer
        if self.enable_perf_logging:
            self.fps_timer = self.create_timer(5.0, self.log_performance)

    def extract_fisheye_parameters(self, camera_info: CameraInfo) -> bool:
        """
        Extract fisheye calibration parameters from CameraInfo message
        Returns True if parameters were successfully extracted and are different from current
        """
        try:
            # Extract camera matrix (3x3)
            K = np.array(camera_info.k).reshape(3, 3)
            
            # Extract distortion coefficients - FISHEYE RICHIEDE ESATTAMENTE 4 COEFFICIENTI
            D_full = np.array(camera_info.d)
            
            # Per fisheye OpenCV, usa solo i primi 4 coefficienti
            if len(D_full) >= 4:
                D = D_full[:4].astype(np.float64)  # Prendi solo i primi 4
            else:
                self.get_logger().error(f'Fisheye distortion requires at least 4 coefficients, got {len(D_full)}')
                return False
                   
            # Validate fisheye distortion model
            if camera_info.distortion_model != 'fisheye':
                if not hasattr(self, '_distortion_warning_shown'):
                    self.get_logger().warn(
                        f'Camera distortion model is "{camera_info.distortion_model}", '
                        'expected "fisheye". Proceeding anyway.'
                    )
                    self._distortion_warning_shown = True
            
            # Check if parameters have changed - CONTROLLO PRIMA DEI LOG!
            current_image_size = (camera_info.width, camera_info.height)
            if (self.camera_matrix is not None and 
                np.allclose(K, self.camera_matrix) and 
                np.allclose(D, self.dist_coeffs) and 
                self.image_size == current_image_size):
                return False  # No change in parameters 
    
            self.get_logger().info(f'Using first 4 fisheye coefficients: {D}')
            if len(D_full) > 4:
                self.get_logger().warn(f'Ignoring extra distortion coefficients: {D_full[4:]}')
                    
            # Update parameters
            self.camera_matrix = K
            self.dist_coeffs = D 
            self.image_size = current_image_size
            self.get_logger().info('Fisheye calibration parameters updated:')
            self.get_logger().info(f'  Image size: {self.image_size}')
            self.get_logger().info(f'  Camera matrix fx={K[0,0]:.1f}, fy={K[1,1]:.1f}')
            self.get_logger().info(f'  Principal point: ({K[0,2]:.1f}, {K[1,2]:.1f})')
            self.get_logger().info(f'  Distortion coeffs (4): {D}')
            return True
                
        except Exception as e:
            self.get_logger().error(f'Failed to extract fisheye parameters: {str(e)}')
            return False


    def setup_fisheye_correction(self) -> bool:
        """
        Setup fisheye correction maps for efficient undistortion
        Returns True if setup was successful
        """
        if self.camera_matrix is None or self.dist_coeffs is None or self.image_size is None:
            return False
    
        try:
            w, h = self.image_size
            
            # Estimate new camera matrix for undistorted image (FISHEYE VERSION)
            new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.camera_matrix, 
                self.dist_coeffs, 
                (w, h), 
                np.eye(3), 
                balance=self.fisheye_alpha,  # balance parameter (0.0-1.0)
                new_size=(w, h)
            )
    
            # Create undistortion maps for efficient processing
            self.undistort_map1, self.undistort_map2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix, 
                self.dist_coeffs, 
                np.eye(3), 
                new_camera_matrix, 
                (w, h), 
                cv2.CV_16SC2
            )
    
            # Store new camera matrix for depth camera info
            self.corrected_camera_matrix = new_camera_matrix
            
            self.fisheye_correction_ready = True
            self.get_logger().info('Fisheye correction maps initialized successfully')
            return True
    
        except Exception as e:
            self.get_logger().error(f'Failed to setup fisheye correction: {str(e)}')
            self.fisheye_correction_ready = False
            return False

    def correct_fisheye_distortion(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Apply fisheye distortion correction using pre-computed maps
        """
        if not self.fisheye_correction_ready or self.undistort_map1 is None:
            return cv_image

        try:
            # Apply undistortion using pre-computed maps (very efficient)
            undistorted = cv2.remap(
                cv_image, 
                self.undistort_map1, 
                self.undistort_map2, 
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT
            )

            # Optional: crop to remove black borders
            if self.crop_corrected:
                undistorted = self.crop_center_region(undistorted, self.crop_ratio)

            return undistorted

        except Exception as e:
            self.get_logger().error(f'Fisheye correction failed: {str(e)}')
            return cv_image

    def crop_center_region(self, image: np.ndarray, crop_ratio: float) -> np.ndarray:
        """Crop center region of image to remove fisheye correction artifacts"""
        h, w = image.shape[:2]
        crop_h, crop_w = int(h * crop_ratio), int(w * crop_ratio)
        start_y, start_x = (h - crop_h) // 2, (w - crop_w) // 2
        return image[start_y:start_y + crop_h, start_x:start_x + crop_w]

    def preprocess_endoscopic_image(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Complete preprocessing pipeline for endoscopic fisheye images
        """
        # Step 1: Apply fisheye distortion correction (most critical step)
        if self.enable_fisheye_correction and self.fisheye_correction_ready:
            cv_image = self.correct_fisheye_distortion(cv_image)

        # Step 2: Downsample if requested (for performance)
        if self.downsample_factor > 1:
            h, w = cv_image.shape[:2]
            new_h, new_w = h // self.downsample_factor, w // self.downsample_factor
            cv_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # Step 3: Apply endoscopic enhancements
        if self.endoscopic_preprocessing:
            cv_image = self.apply_endoscopic_enhancements(cv_image)

        return cv_image

    def apply_endoscopic_enhancements(self, cv_image: np.ndarray) -> np.ndarray:
        """Apply enhancements specific to endoscopic imaging"""
        try:
            # Convert to LAB color space for better illumination control
            lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
            
            # Apply CLAHE to L channel (luminance) to handle uneven illumination
            lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
            
            # Convert back to BGR
            enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            
            # Reduce specular highlights (common in wet biological surfaces)
            enhanced = self.reduce_specular_highlights(enhanced)
            
            return enhanced

        except Exception as e:
            self.get_logger().error(f'Endoscopic enhancement failed: {str(e)}')
            return cv_image

    def reduce_specular_highlights(self, image: np.ndarray) -> np.ndarray:
        """Reduce specular highlights on wet surfaces using inpainting"""
        try:
            # Convert to grayscale for highlight detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Create mask for bright specular regions
            _, highlight_mask = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
            
            # Morphological operations to clean up mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            highlight_mask = cv2.morphologyEx(highlight_mask, cv2.MORPH_CLOSE, kernel)
            
            # Inpaint specular regions
            if np.any(highlight_mask):
                inpainted = cv2.inpaint(image, highlight_mask, 3, cv2.INPAINT_TELEA)
                # Blend with original (gentle correction)
                alpha = 0.7
                result = cv2.addWeighted(image, alpha, inpainted, 1 - alpha, 0)
                return result
            
            return image

        except Exception as e:
            self.get_logger().error(f'Specular highlight reduction failed: {str(e)}')
            return image

    def estimate_depth(self, cv_image: np.ndarray) -> Optional[np.ndarray]:
        """Perform depth estimation using MiDaS with optimized processing"""
        try:
            # Preprocess image (includes fisheye correction and enhancements)
            processed_image = self.preprocess_endoscopic_image(cv_image)
            
            # Apply MiDaS transforms
            input_tensor = self.midas_transform({"image": processed_image})["image"]
            
            # Move to device and add batch dimension
            input_tensor = torch.from_numpy(input_tensor).to(self.device).unsqueeze(0)
            
            # Use half precision if enabled
            if self.use_half_precision and self.device.type == 'cuda':
                input_tensor = input_tensor.half()

            # Perform inference
            with torch.no_grad():
                depth_tensor = self.model(input_tensor)

            # Convert to numpy
            depth = depth_tensor.squeeze().cpu().numpy().astype(np.float32)

            # Post-process depth for the application
            depth_final = self.postprocess_depth(depth, cv_image.shape)

            return depth_final

        except Exception as e:
            self.get_logger().error(f'Depth estimation failed: {str(e)}')
            return None

    def postprocess_depth(self, depth: np.ndarray, original_shape: Tuple[int, int, int]) -> np.ndarray:
        """Post-process depth map for tabletop object reconstruction"""
        try:
            # Resize depth to match original image size
            h, w = original_shape[:2]
            if self.enable_fisheye_correction and self.crop_corrected:
                # Adjust for cropping
                crop_h, crop_w = int(h * self.crop_ratio), int(w * self.crop_ratio)
                depth_resized = cv2.resize(depth, (crop_w, crop_h), interpolation=cv2.INTER_CUBIC)
                
                # Pad back to original size
                pad_y = (h - crop_h) // 2
                pad_x = (w - crop_w) // 2
                depth_final = np.zeros((h, w), dtype=np.float32)
                depth_final[pad_y:pad_y+crop_h, pad_x:pad_x+crop_w] = depth_resized
            else:
                depth_final = cv2.resize(depth, (w, h), interpolation=cv2.INTER_CUBIC)

            # Normalize depth values
            depth_normalized = cv2.normalize(depth_final, None, 0, 1, cv2.NORM_MINMAX)

            # Convert to metric depth (MiDaS produces inverse depth)
            # Scale to realistic range for tabletop objects (50mm to 500mm)
            depth_range = self.max_depth - self.min_depth
            depth_metric = self.min_depth + depth_range * (1.0 - depth_normalized)

            # Apply smoothing for better surface reconstruction
            depth_smooth = cv2.bilateralFilter(
                depth_metric.astype(np.float32), 9, 75, 75
            )

            # Convert to uint16 format for ROS (depth in mm, scaled by depth_scale)
            depth_uint16 = (depth_smooth * self.depth_scale / 1000.0).astype(np.uint16)

            return depth_uint16

        except Exception as e:
            self.get_logger().error(f'Depth post-processing failed: {str(e)}')
            return None

    def create_depth_camera_info(self, original_camera_info: CameraInfo) -> CameraInfo:
        """Create camera info for depth image, accounting for fisheye correction"""
        depth_camera_info = CameraInfo()
        depth_camera_info.header = original_camera_info.header
        depth_camera_info.height = original_camera_info.height
        depth_camera_info.width = original_camera_info.width

        if self.enable_fisheye_correction and self.fisheye_correction_ready:
            # Use corrected camera matrix
            depth_camera_info.k = self.corrected_camera_matrix.flatten().tolist()
            depth_camera_info.distortion_model = "plumb_bob"  # No distortion after correction
            depth_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        else:
            # Use original parameters
            depth_camera_info.k = original_camera_info.k
            depth_camera_info.distortion_model = original_camera_info.distortion_model
            depth_camera_info.d = original_camera_info.d

        # Copy other parameters
        depth_camera_info.r = original_camera_info.r
        depth_camera_info.p = original_camera_info.p

        return depth_camera_info

    def synchronized_callback(self, image_msg: Image, camera_info_msg: CameraInfo):
        """Main synchronized callback for image and camera info"""
        try:
            # Preserve original timestamps
            original_timestamp = image_msg.header.stamp
            original_frame_id = image_msg.header.frame_id

            # Extract and update fisheye parameters if auto-extraction is enabled
            if self.auto_extract_params and self.enable_fisheye_correction:
                if self.extract_fisheye_parameters(camera_info_msg):
                    # Parameters changed, setup new correction maps
                    self.setup_fisheye_correction()

            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            if cv_image is None:
                self.get_logger().error("Failed to convert ROS image to OpenCV format")
                return

            # Estimate depth
            depth_array = self.estimate_depth(cv_image)
            if depth_array is None:
                self.get_logger().error("Depth estimation returned None")
                return

            # Create and publish depth image
            depth_msg = self.bridge.cv2_to_imgmsg(depth_array, "16UC1")
            depth_msg.header.stamp = original_timestamp
            depth_msg.header.frame_id = original_frame_id
            self.depth_pub.publish(depth_msg)

            # Create and publish depth camera info
            depth_camera_info = self.create_depth_camera_info(camera_info_msg)
            depth_camera_info.header.stamp = original_timestamp
            depth_camera_info.header.frame_id = original_frame_id
            self.depth_info_pub.publish(depth_camera_info)

            # Optional: publish corrected RGB image for visualization
            if self.enable_fisheye_correction and self.fisheye_correction_ready:
                corrected_rgb = self.correct_fisheye_distortion(cv_image)
                corrected_msg = self.bridge.cv2_to_imgmsg(corrected_rgb, 'bgr8')
                corrected_msg.header.stamp = original_timestamp
                corrected_msg.header.frame_id = original_frame_id
                self.corrected_image_pub.publish(corrected_msg)

            # Update performance counter
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Synchronized callback failed: {str(e)}')

    def log_performance(self):
        """Log performance metrics"""
        if not self.enable_perf_logging:
            return

        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_fps_time).nanoseconds / 1e9

        if elapsed > 0 and self.frame_count > 0:
            fps = self.frame_count / elapsed
            self.get_logger().info(
                f'Performance: {fps:.2f} FPS | '
                f'Fisheye correction: {"ON" if self.fisheye_correction_ready else "OFF"} | '
                f'Device: {self.device} | '
                f'Half precision: {self.use_half_precision}'
            )

        # Reset counters
        self.frame_count = 0
        self.last_fps_time = current_time

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = MidasFisheyeDepthNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Node failed with error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

