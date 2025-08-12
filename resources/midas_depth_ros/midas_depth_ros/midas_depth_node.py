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
    Optimized for RTX 3070 and endoscopic bladder mapping with fisheye correction
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
        self.declare_parameter('max_depth', 2000.0)  # mm, more realistic for endoscope
        self.declare_parameter('endoscope_mask_path', '/root/endoscope_mask.png')
        
        # Get parameters
        self.model_type = self.get_parameter('model_type').value
        self.input_res = self.get_parameter('input_resolution').value
        self.downsample_factor = self.get_parameter('camera_downsample_factor').value
        self.optimize_transforms = self.get_parameter('optimize_transforms').value
        self.use_half_precision = self.get_parameter('use_half_precision').value
        self.vesica_preprocessing = self.get_parameter('apply_vesica_preprocessing').value
        self.depth_scale = self.get_parameter('depth_scale_factor').value
        self.max_depth = self.get_parameter('max_depth').value
        self.mask_path = self.get_parameter('endoscope_mask_path').value
        
        # Initialize fisheye correction maps
        self.undistort_maps = None
        
        # Load endoscope mask
        self.endoscope_mask = None
        if os.path.exists(self.mask_path):
            self.endoscope_mask = cv2.imread(self.mask_path, cv2.IMREAD_GRAYSCALE)
            self.get_logger().info(f'Loaded endoscope mask: {self.mask_path}')
        else:
            self.get_logger().warn(f'Endoscope mask not found: {self.mask_path}')
        
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
        
        self.get_logger().info('MiDaS Depth Node with fisheye correction initialized successfully')
    
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
    
    def undistort_fisheye(self, image, camera_info):
        """Corregge distorsione fisheye per MiDaS"""
        K = np.array(camera_info.k).reshape(3, 3)
        D = np.array(camera_info.d)
        
        h, w = image.shape[:2]
        
        # Crea mappa di correzione fisheye una volta sola per performance
        if self.undistort_maps is None:
            self.undistort_maps = cv2.fisheye.initUndistortRectifyMap(
                K, D, np.eye(3), K, (w, h), cv2.CV_16SC2
            )
            self.get_logger().info(f'Created fisheye undistort maps for {w}x{h}')
        
        map1, map2 = self.undistort_maps
        
        # Applica correzione
        undistorted = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
        return undistorted
    
    def redistort_depth_to_raw(self, depth_corrected, camera_info, raw_shape):
        """Ri-distorce depth per matchare geometria raw"""
        K = np.array(camera_info.k).reshape(3, 3)
        D = np.array(camera_info.d)
        
        h, w = raw_shape[:2]
        
        try:
            # Crea griglia di punti raw
            y_raw, x_raw = np.mgrid[0:h, 0:w].astype(np.float32)
            points_raw = np.stack([x_raw.ravel(), y_raw.ravel()], axis=1)
            
            # Correggi i punti raw per ottenere coordinate corrette
            points_corrected = cv2.fisheye.undistortPoints(
                points_raw.reshape(-1, 1, 2), K, D, P=K
            ).reshape(-1, 2)
            
            # Interpola depth dalle coordinate corrette
            x_corr = points_corrected[:, 0].reshape(h, w)
            y_corr = points_corrected[:, 1].reshape(h, w)
            
            # Assicurati che depth_corrected abbia le stesse dimensioni
            if depth_corrected.shape != (h, w):
                depth_corrected = cv2.resize(depth_corrected, (w, h), interpolation=cv2.INTER_CUBIC)
            
            # Interpola depth usando remap
            depth_raw = cv2.remap(
                depth_corrected, x_corr, y_corr, 
                cv2.INTER_LINEAR, borderValue=0
            )
            
            return depth_raw
            
        except Exception as e:
            self.get_logger().error(f'Redistortion failed: {str(e)}')
            # Fallback: resize semplice
            return cv2.resize(depth_corrected, (w, h), interpolation=cv2.INTER_CUBIC)
    
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
    
    def estimate_depth(self, cv_image, camera_info):
        """Depth estimation con correzione fisheye e ri-mappatura"""
        try:
            # 1. CORREGGI FISHEYE per MiDaS
            corrected_image = self.undistort_fisheye(cv_image, camera_info)
            
            # 2. MiDaS su immagine CORRETTA
            processed_image = self.preprocess_endoscopic_image(corrected_image)
            input_tensor = self.transform({"image": processed_image})["image"]
            
            input_tensor = torch.from_numpy(input_tensor).to(self.device).unsqueeze(0)
            
            if self.use_half_precision and self.device.type == 'cuda':
                input_tensor = input_tensor.half()
            
            # Inference su geometria corretta
            with torch.no_grad():
                depth_tensor = self.model(input_tensor)
            
            depth_corrected = depth_tensor.squeeze().cpu().numpy().astype(np.float32)
            
            # 3. Post-process depth corretta
            depth_processed = self.postprocess_depth_corrected(depth_corrected, corrected_image.shape)
            
            # 4. RI-DISTORCI per matchare RGB raw
            depth_raw = self.redistort_depth_to_raw(depth_processed, camera_info, cv_image.shape)
            
            return depth_raw
            
        except Exception as e:
            self.get_logger().error(f'Depth estimation failed: {str(e)}')
            return None
    
    def postprocess_depth_corrected(self, depth, corrected_shape):
        """Post-process su depth corretta (senza ridistorsione)"""
        # Resize alla forma corretta
        depth_resized = cv2.resize(
            depth, 
            (corrected_shape[1], corrected_shape[0]), 
            interpolation=cv2.INTER_CUBIC
        )
        
        # Normalizza e scala
        depth_normalized = cv2.normalize(depth_resized, None, 0, 1, cv2.NORM_MINMAX)
        depth_mm = self.max_depth * (1.0 - depth_normalized)
        
        # Smooth su geometria corretta
        depth_smooth = cv2.medianBlur(depth_mm.astype(np.float32), 5)
        
        return depth_smooth
    
    def apply_endoscope_mask(self, depth_array):
        """Applica maschera endoscopio su depth raw"""
        if self.endoscope_mask is not None:
            if self.endoscope_mask.shape != depth_array.shape[:2]:
                mask_resized = cv2.resize(
                    self.endoscope_mask, 
                    (depth_array.shape[1], depth_array.shape[0])
                )
            else:
                mask_resized = self.endoscope_mask
            
            # Maschera: 0 dove maschera è nera
            depth_masked = np.where(mask_resized > 127, depth_array, 0)
            return depth_masked
        
        return depth_array
    
    def postprocess_depth(self, depth, original_shape):
        """DEPRECATED: Metodo originale mantenuto per compatibilità"""
        # Questo metodo non viene più usato, ma mantenuto per sicurezza
        depth_resized = cv2.resize(
            depth, 
            (original_shape[1], original_shape[0]), 
            interpolation=cv2.INTER_CUBIC
        )
        
        depth_normalized = cv2.normalize(depth_resized, None, 0, 255, cv2.NORM_MINMAX)
        depth_mm = self.max_depth * (1.0 - depth_normalized / 255.0)
        depth_smooth = cv2.medianBlur(depth_mm.astype(np.float32), 5)
        
        # CORREZIONE: Non più la scala sbagliata
        depth_final = depth_smooth.astype(np.uint16)
        
        return depth_final
    
    def image_callback(self, image_msg, camera_info_msg):
        """Callback principale con correzione fisheye"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
            
            if cv_image is None:
                self.get_logger().error("Image conversion failed!")
                return
            
            # Stima depth con pipeline corretta (fisheye correction)
            depth_raw = self.estimate_depth(cv_image, camera_info_msg)
            
            if depth_raw is not None:
                # Applica maschera endoscopio su risultato raw
                depth_masked = self.apply_endoscope_mask(depth_raw)
                
                # Pubblica depth raw (quello che vuole RTABMap)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_masked.astype(np.uint16), "16UC1")
                depth_msg.header = image_msg.header
                
                self.depth_pub.publish(depth_msg)
                
                # Camera info raw (identica all'input)
                camera_info_msg.header = image_msg.header
                self.depth_info_pub.publish(camera_info_msg)
                
                self.frame_count += 1
                
        except Exception as e:
            self.get_logger().error(f'Callback failed: {str(e)}')
    
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