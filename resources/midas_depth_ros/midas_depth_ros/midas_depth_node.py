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

# Add MiDaS to path
sys.path.append('/midas')
from midas.model_loader import load_model, default_models
from midas.midas_net import MidasNet
from midas.transforms import Resize, NormalizeImage, PrepareForNet


class MidasDepthNode(Node):
    """
    ROS2 node for real-time depth estimation using MiDaS
    Optimized for RTX 3070 and endoscopic bladder mapping
    """
    
    def __init__(self):
        super().__init__('midas_depth_node')
        
        # Declare parameters
        self.declare_parameter('model_type', 'dpt_hybrid_384')
        self.declare_parameter('input_resolution', [384, 384])
        self.declare_parameter('camera_downsample_factor', 2)
        self.declare_parameter('optimize_transforms', True)
        self.declare_parameter('use_half_precision', True)
        self.declare_parameter('apply_vesica_preprocessing', True)
        self.declare_parameter('depth_scale_factor', 1000.0)  # Convert to mm
        self.declare_parameter('max_depth', 200.0)  # mm, typical for bladder
        
        # Get parameters
        self.model_type = self.get_parameter('model_type').value
        self.input_res = self.get_parameter('input_resolution').value
        self.downsample_factor = self.get_parameter('camera_downsample_factor').value
        self.optimize_transforms = self.get_parameter('optimize_transforms').value
        self.use_half_precision = self.get_parameter('use_half_precision').value
        self.vesica_preprocessing = self.get_parameter('apply_vesica_preprocessing').value
        self.depth_scale = self.get_parameter('depth_scale_factor').value
        self.max_depth = self.get_parameter('max_depth').value
        
        # Initialize device (RTX 3070 optimization)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            self.get_logger().info(f'Using GPU: {torch.cuda.get_device_name()}')
            torch.backends.cudnn.benchmark = True  # Optimize for RTX 3070
        else:
            self.get_logger().warn('CUDA not available, using CPU')
        
        # Load MiDaS model
        self.load_model()
        
        # Setup transforms for endoscopic images
        self.setup_transforms()
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Setup publishers and subscribers
        self.setup_ros_interface()
        
        self.get_logger().info('MiDaS Depth Node initialized successfully')
    
    def load_model(self):
        """Load and optimize MiDaS model"""
        try:
            self.get_logger().info(f'Loading MiDaS model: {self.model_type}')
            
            # Use torch.hub instead of local files
            if self.model_type == 'dpt_hybrid_384':
                self.model = torch.hub.load('intel-isl/MiDaS', 'DPT_Hybrid', pretrained=True, trust_repo=True)
            elif self.model_type == 'dpt_large_384':
                self.model = torch.hub.load('intel-isl/MiDaS', 'DPT_Large', pretrained=True, trust_repo=True)
            elif self.model_type == 'midas_v21_small_256':
                self.model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small', pretrained=True, trust_repo=True)
            else:
                self.get_logger().error(f'Unsupported model type: {self.model_type}')
                raise ValueError(f'Unsupported model type: {self.model_type}')
            
            # Move to device
            self.model.to(self.device)
            self.model.eval()
            
            # Use half precision on GPU
            if self.use_half_precision and self.device.type == 'cuda':
                self.model = self.model.half()
                self.get_logger().info('Using half precision (FP16)')
            
            self.get_logger().info(f'MiDaS model {self.model_type} loaded successfully on {self.device}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load MiDaS model: {str(e)}')
            raise
    
    def setup_transforms(self):
        """Setup image transforms optimized for endoscopic images"""
        # Standard MiDaS transforms
        self.transform = transforms.Compose([
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
        if self.vesica_preprocessing:
            self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    
    def setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers"""
        self.image_sub = message_filters.Subscriber(
            self, Image, '/endoscope/image_raw'  
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/endoscope/camera_info'  
        )

        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], 
            queue_size=5,  
            slop=0.03      
        )
        self.sync.registerCallback(self.image_callback)

        # Publishers - mantengono i nomi standard per RTAB-Map
        self.depth_pub = self.create_publisher(
            Image, '/endoscope/depth/image_raw', 10
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/endoscope/depth/camera_info', 10
        )
    
        # Performance monitoring
        self.fps_timer = self.create_timer(5.0, self.log_performance)
        self.frame_count = 0
        self.last_fps_time = self.get_clock().now()
    
    def preprocess_endoscopic_image(self, cv_image):
        """Preprocessing optimized for endoscopic bladder images"""

        # NUOVO: Downsample per velocizzare se richiesto
        if self.downsample_factor > 1:
            h, w = cv_image.shape[:2]
            new_h, new_w = h // self.downsample_factor, w // self.downsample_factor
            cv_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_AREA)

        if not self.vesica_preprocessing:
            return cv_image

        # Convert to LAB for better illumination correction
        lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

        # Apply CLAHE to L channel (illumination)
        lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])

        # Convert back to BGR
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # Reduce specular highlights (common in wet bladder surfaces)
        enhanced = self.reduce_specular_highlights(enhanced)
    
        return enhanced
    
    def reduce_specular_highlights(self, image):
        """Reduce specular highlights on wet bladder surfaces"""
        # Simple specular highlight reduction
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
        
        # Inpaint specular regions
        inpainted = cv2.inpaint(image, mask, 3, cv2.INPAINT_TELEA)
        
        # Blend with original
        alpha = 0.7
        result = cv2.addWeighted(image, alpha, inpainted, 1-alpha, 0)
        
        return result
    
    def estimate_depth(self, cv_image):
        """Perform depth estimation using MiDaS"""
        try:
            # Preprocess for endoscopy
            processed_image = self.preprocess_endoscopic_image(cv_image)
            
            # Apply MiDaS transforms
            input_tensor = self.transform({"image": processed_image})["image"]
            
            # Move to device and add batch dimension
            input_tensor = torch.from_numpy(input_tensor).to(self.device).unsqueeze(0)
            
            # Use half precision if enabled
            if self.use_half_precision and self.device.type == 'cuda':
                input_tensor = input_tensor.half()
            
            # Inference
            with torch.no_grad():
                depth_tensor = self.model(input_tensor)
            
            # Convert to numpy
            depth = depth_tensor.squeeze().cpu().numpy()
            
            # Post-process for bladder mapping
            depth = self.postprocess_depth(depth, cv_image.shape)
            
            return depth
            
        except Exception as e:
            self.get_logger().error(f'Depth estimation failed: {str(e)}')
            return None
    
    def postprocess_depth(self, depth, original_shape):
        """Post-process depth map for bladder mapping"""
        # Resize to original image size
        depth_resized = cv2.resize(
            depth, 
            (original_shape[1], original_shape[0]), 
            interpolation=cv2.INTER_CUBIC
        )
        
        # Normalize and scale for bladder dimensions
        depth_normalized = cv2.normalize(depth_resized, None, 0, 255, cv2.NORM_MINMAX)
        
        # Convert to appropriate depth values (mm)
        # Invert because MiDaS gives inverse depth
        depth_mm = self.max_depth * (1.0 - depth_normalized / 255.0)
        
        # Apply median filter to smooth biological surfaces
        depth_smooth = cv2.medianBlur(depth_mm.astype(np.float32), 5)
        
        # Convert to uint16 for ROS depth message (depth in mm)
        depth_final = (depth_smooth * self.depth_scale / self.max_depth).astype(np.uint16)
        
        return depth_final
    
    def image_callback(self, image_msg, camera_info_msg):
        """Main callback for synchronized image and camera info"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Estimate depth
            depth_array = self.estimate_depth(cv_image)
            
            if depth_array is not None:
                # Create depth image message
                depth_msg = self.bridge.cv2_to_imgmsg(depth_array, "16UC1")
                depth_msg.header = image_msg.header
                
                # Publish depth
                self.depth_pub.publish(depth_msg)
                
                # Publish depth camera info (same as RGB camera)
                camera_info_msg.header = image_msg.header
                self.depth_info_pub.publish(camera_info_msg)
                
                # Update performance counter
                self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Image callback failed: {str(e)}')
    
    def log_performance(self):
        """Log performance metrics"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f'Depth estimation FPS: {fps:.2f}')
        
        self.frame_count = 0
        self.last_fps_time = current_time


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MidasDepthNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()