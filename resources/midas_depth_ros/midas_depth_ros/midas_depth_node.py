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
    Publishes both corrected and raw-distorted depth maps
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
        
        self.get_logger().info('MiDaS Depth Node with dual depth publication initialized successfully')
    
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

        # DOPPIA PUBBLICAZIONE DEPTH:
        # 1. Depth RAW distorta (per RTABMap con immagine raw)
        self.depth_raw_pub = self.create_publisher(
            Image, '/endoscope/depth/image_raw', 10
        )
        self.depth_raw_info_pub = self.create_publisher(
            CameraInfo, '/endoscope/depth/camera_info', 10
        )
        
        # 2. Depth CORRETTA (geometria fisheye corretta, qualità migliore)
        self.depth_corrected_pub = self.create_publisher(
            Image, '/endoscope/depth_corrected/image_raw', 10
        )
        self.depth_corrected_info_pub = self.create_publisher(
            CameraInfo, '/endoscope/depth_corrected/camera_info', 10
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
    
    def create_corrected_camera_info(self, original_camera_info):
        """Crea camera_info per l'immagine corretta (senza distorsione)"""
        corrected_info = CameraInfo()
        corrected_info.header = original_camera_info.header
        corrected_info.height = original_camera_info.height
        corrected_info.width = original_camera_info.width
        corrected_info.distortion_model = "plumb_bob"  # Nessuna distorsione fisheye
        
        # Mantieni matrice intrinseca K
        corrected_info.k = original_camera_info.k
        
        # Azzera coefficienti di distorsione (immagine corretta)
        corrected_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Matrice di proiezione e rettifica
        corrected_info.p = original_camera_info.p
        corrected_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identità
        
        return corrected_info
    
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
    
    def estimate_depth_dual(self, cv_image, camera_info):
        """
        Stima depth UNA SOLA VOLTA su immagine corretta e ritorna ENTRAMBE le versioni:
        - depth_corrected: su geometria corretta (migliore qualità)
        - depth_raw: ri-distorta per matchare immagine raw (per RTABMap)
        """
        try:
            # 1. CORREGGI FISHEYE per MiDaS (una sola volta)
            corrected_image = self.undistort_fisheye(cv_image, camera_info)
            
            # 2. MiDaS SOLO su immagine CORRETTA (una sola volta)
            processed_image = self.preprocess_endoscopic_image(corrected_image)
            input_tensor = self.transform({"image": processed_image})["image"]
            
            input_tensor = torch.from_numpy(input_tensor).to(self.device).unsqueeze(0)
            
            if self.use_half_precision and self.device.type == 'cuda':
                input_tensor = input_tensor.half()
            
            # Inference UNA SOLA VOLTA su geometria corretta
            with torch.no_grad():
                depth_tensor = self.model(input_tensor)
            
            depth_midas_raw = depth_tensor.squeeze().cpu().numpy().astype(np.float32)
            
            # 3. Post-process depth corretta (questa è la versione "buona")
            depth_corrected = self.postprocess_depth_corrected(depth_midas_raw, corrected_image.shape)
            
            # 4. RI-DISTORCI solo il RISULTATO depth per matchare RGB raw
            depth_raw = self.redistort_depth_to_raw(depth_corrected, camera_info, cv_image.shape)
            
            return depth_corrected, depth_raw
            
        except Exception as e:
            self.get_logger().error(f'Depth estimation failed: {str(e)}')
            return None, None
    
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
    
    def apply_endoscope_mask(self, depth_array, target_shape=None):
        """Applica maschera endoscopio"""
        if self.endoscope_mask is not None:
            # Se specificato target_shape, usa quello, altrimenti usa shape depth
            if target_shape is not None:
                mask_shape = target_shape[:2]
            else:
                mask_shape = depth_array.shape[:2]
                
            if self.endoscope_mask.shape != mask_shape:
                mask_resized = cv2.resize(
                    self.endoscope_mask, 
                    (mask_shape[1], mask_shape[0])
                )
            else:
                mask_resized = self.endoscope_mask
            
            # Maschera: 0 dove maschera è nera
            depth_masked = np.where(mask_resized > 127, depth_array, 0)
            return depth_masked
        
        return depth_array
    
    def image_callback(self, image_msg, camera_info_msg):
        """Callback principale con doppia pubblicazione depth"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
            
            if cv_image is None:
                self.get_logger().error("Image conversion failed!")
                return
            
            # Stima ENTRAMBE le depth
            depth_corrected, depth_raw = self.estimate_depth_dual(cv_image, camera_info_msg)
            
            if depth_corrected is not None and depth_raw is not None:
                
                # === PUBBLICAZIONE 1: DEPTH RAW (per RTABMap) ===
                depth_raw_masked = self.apply_endoscope_mask(depth_raw, cv_image.shape)
                depth_raw_msg = self.bridge.cv2_to_imgmsg(depth_raw_masked.astype(np.uint16), "16UC1")
                depth_raw_msg.header = image_msg.header
                self.depth_raw_pub.publish(depth_raw_msg)
                
                # Camera info raw (identica all'input, con distorsione fisheye)
                camera_info_raw = camera_info_msg
                camera_info_raw.header = image_msg.header
                self.depth_raw_info_pub.publish(camera_info_raw)
                
                # === PUBBLICAZIONE 2: DEPTH CORRECTED (qualità migliore) ===
                depth_corrected_masked = self.apply_endoscope_mask(depth_corrected)
                depth_corrected_msg = self.bridge.cv2_to_imgmsg(depth_corrected_masked.astype(np.uint16), "16UC1")
                depth_corrected_msg.header = image_msg.header
                self.depth_corrected_pub.publish(depth_corrected_msg)
                
                # Camera info corrected (senza distorsione fisheye)
                camera_info_corrected = self.create_corrected_camera_info(camera_info_msg)
                camera_info_corrected.header = image_msg.header
                self.depth_corrected_info_pub.publish(camera_info_corrected)
                
                self.frame_count += 1
                
        except Exception as e:
            self.get_logger().error(f'Callback failed: {str(e)}')
    
    def log_performance(self):
        """Log performance metrics"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f'Dual depth estimation FPS: {fps:.2f}')
        
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