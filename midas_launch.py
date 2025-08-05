
#!/usr/bin/env python3

"""
Complete ROS2 Launch File for MiDaS Fisheye Depth Estimation Node
Optimized for endoscopic imaging with fisheye lens correction
Compatible with RTAB-Map integration and UR3 robotic setup
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with all parameters for MiDaS fisheye depth node"""
    
    # Package and executable information
    package_name = 'midas_depth_ros'  
    executable_name = 'midas_depth_node.py'
    node_name = 'midas_depth_node'
    
    # ========================================
    # ORIGINAL MIDAS PARAMETERS (Backward Compatibility)
    # ========================================
    
    declare_model_type = DeclareLaunchArgument(
        'model_type',
        default_value='dpt_hybrid_384',
        description='MiDaS model type to use for depth estimation. Options: '
                   'dpt_hybrid_384 (best balance), dpt_large_384 (highest accuracy), '
                   'midas_v21_small_256 (fastest)'
    )
    
    declare_input_resolution = DeclareLaunchArgument(
        'input_resolution',
        default_value='[384, 384]',
        description='Input resolution for MiDaS model [width, height]. '
                   'Higher resolution = better accuracy but slower processing'
    )
    
    declare_camera_downsample_factor = DeclareLaunchArgument(
        'camera_downsample_factor',
        default_value='1',
        description='Downsample factor for input camera image. '
                   '1=no downsampling, 2=half resolution (faster processing)'
    )
    
    declare_optimize_transforms = DeclareLaunchArgument(
        'optimize_transforms',
        default_value='true',
        description='Enable MiDaS transform optimizations for better performance'
    )
    
    declare_use_half_precision = DeclareLaunchArgument(
        'use_half_precision',
        default_value='true',
        description='Use FP16 half precision on GPU for faster inference (RTX 3070 optimized)'
    )
    
    declare_depth_scale_factor = DeclareLaunchArgument(
        'depth_scale_factor',
        default_value='1000.0',
        description='Scale factor for depth values in output message (typically 1000 for mm)'
    )
    
    declare_max_depth = DeclareLaunchArgument(
        'max_depth',
        default_value='500.0',
        description='Maximum depth value in mm (optimized for tabletop objects: 500mm)'
    )
    
    declare_min_depth = DeclareLaunchArgument(
        'min_depth',
        default_value='80.0',
        description='Minimum depth value in mm (prevents too-close objects: 80mm)'
    )
    
    # ========================================
    # FISHEYE CORRECTION PARAMETERS (New)
    # ========================================
    
    declare_enable_fisheye_correction = DeclareLaunchArgument(
        'enable_fisheye_correction',
        default_value='true',
        description='Enable automatic fisheye distortion correction using camera calibration'
    )
    
    declare_fisheye_alpha = DeclareLaunchArgument(
        'fisheye_alpha',
        default_value='0.7',
        description='Fisheye correction alpha parameter (0.0-1.0). '
                   '0.0=no black pixels, 1.0=preserve all source pixels. '
                   '0.7 is optimal for endoscopes'
    )
    
    declare_auto_extract_camera_params = DeclareLaunchArgument(
        'auto_extract_camera_params',
        default_value='true',
        description='Automatically extract fisheye parameters from camera_info messages'
    )
    
    declare_crop_corrected_image = DeclareLaunchArgument(
        'crop_corrected_image',
        default_value='true',
        description='Crop corrected image to remove black borders from fisheye correction'
    )
    
    declare_crop_ratio = DeclareLaunchArgument(
        'crop_ratio',
        default_value='0.85',
        description='Crop ratio for fisheye corrected image (0.8-0.9 recommended). '
                   '0.85 removes most artifacts while preserving useful area'
    )
    
    # ========================================
    # ENDOSCOPIC PREPROCESSING PARAMETERS
    # ========================================
    
    declare_apply_endoscopic_preprocessing = DeclareLaunchArgument(
        'apply_endoscopic_preprocessing',
        default_value='true',
        description='Apply endoscopic-specific image enhancements (CLAHE, specular reduction)'
    )
    
    # Backward compatibility: support old parameter name
    declare_apply_vesica_preprocessing = DeclareLaunchArgument(
        'apply_vesica_preprocessing',
        default_value='true',
        description='[DEPRECATED] Use apply_endoscopic_preprocessing instead. '
                   'Apply vesica/endoscopic-specific image enhancements'
    )
    
    declare_clahe_clip_limit = DeclareLaunchArgument(
        'clahe_clip_limit',
        default_value='2.0',
        description='CLAHE clip limit for contrast enhancement. '
                   '2.0 is optimal for endoscopic illumination correction'
    )
    
    declare_clahe_tile_grid_size = DeclareLaunchArgument(
        'clahe_tile_grid_size',
        default_value='[8, 8]',
        description='CLAHE tile grid size [width, height]. '
                   '[8,8] works well for endoscopic image sizes'
    )
    
    # ========================================
    # PERFORMANCE AND DEBUGGING PARAMETERS
    # ========================================
    
    declare_enable_performance_logging = DeclareLaunchArgument(
        'enable_performance_logging',
        default_value='true',
        description='Enable periodic performance logging (FPS, GPU usage, etc.)'
    )
    
    declare_publish_corrected_image = DeclareLaunchArgument(
        'publish_corrected_image',
        default_value='true',
        description='Publish fisheye-corrected RGB image for visualization and debugging'
    )
    
    # ========================================
    # TOPIC REMAPPING PARAMETERS
    # ========================================
    
    declare_input_image_topic = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/endoscope/image_raw',
        description='Input camera image topic name'
    )
    
    declare_input_camera_info_topic = DeclareLaunchArgument(
        'input_camera_info_topic',
        default_value='/endoscope/camera_info',
        description='Input camera info topic name'
    )
    
    declare_output_depth_topic = DeclareLaunchArgument(
        'output_depth_topic',
        default_value='/endoscope/depth/image_raw',
        description='Output depth image topic name (RTAB-Map compatible)'
    )
    
    declare_output_depth_info_topic = DeclareLaunchArgument(
        'output_depth_info_topic',
        default_value='/endoscope/depth/camera_info',
        description='Output depth camera info topic name'
    )
    
    declare_output_corrected_image_topic = DeclareLaunchArgument(
        'output_corrected_image_topic',
        default_value='/endoscope/image_corrected',
        description='Output fisheye-corrected image topic name'
    )
    
    # ========================================
    # ADVANCED CONFIGURATION PARAMETERS
    # ========================================
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS2 log level: debug, info, warn, error, fatal'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set to true when using rosbag replay)'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Optional YAML config file path for additional parameters'
    )
    
    # ========================================
    # NODE DEFINITION WITH PARAMETER MAPPING
    # ========================================
    
    def create_node_with_parameters(context):
        """Create the node with all parameters, including backward compatibility mapping"""
        
        # Get launch configuration values
        model_type = LaunchConfiguration('model_type').perform(context)
        input_resolution = LaunchConfiguration('input_resolution').perform(context)
        camera_downsample_factor = int(LaunchConfiguration('camera_downsample_factor').perform(context))
        optimize_transforms = LaunchConfiguration('optimize_transforms').perform(context).lower() == 'true'
        use_half_precision = LaunchConfiguration('use_half_precision').perform(context).lower() == 'true'
        depth_scale_factor = float(LaunchConfiguration('depth_scale_factor').perform(context))
        max_depth = float(LaunchConfiguration('max_depth').perform(context))
        min_depth = float(LaunchConfiguration('min_depth').perform(context))
        
        # Fisheye parameters
        enable_fisheye_correction = LaunchConfiguration('enable_fisheye_correction').perform(context).lower() == 'true'
        fisheye_alpha = float(LaunchConfiguration('fisheye_alpha').perform(context))
        auto_extract_camera_params = LaunchConfiguration('auto_extract_camera_params').perform(context).lower() == 'true'
        crop_corrected_image = LaunchConfiguration('crop_corrected_image').perform(context).lower() == 'true'
        crop_ratio = float(LaunchConfiguration('crop_ratio').perform(context))
        
        # Endoscopic preprocessing (with backward compatibility)
        apply_endoscopic = LaunchConfiguration('apply_endoscopic_preprocessing').perform(context).lower() == 'true'
        apply_vesica = LaunchConfiguration('apply_vesica_preprocessing').perform(context).lower() == 'true'
        # Use endoscopic if explicitly set, otherwise fall back to vesica for backward compatibility
        apply_preprocessing = apply_endoscopic or apply_vesica
        
        clahe_clip_limit = float(LaunchConfiguration('clahe_clip_limit').perform(context))
        clahe_tile_grid_size = eval(LaunchConfiguration('clahe_tile_grid_size').perform(context))
        
        # Performance parameters
        enable_performance_logging = LaunchConfiguration('enable_performance_logging').perform(context).lower() == 'true'
        publish_corrected_image = LaunchConfiguration('publish_corrected_image').perform(context).lower() == 'true'
        
        # Topic names
        input_image_topic = LaunchConfiguration('input_image_topic').perform(context)
        input_camera_info_topic = LaunchConfiguration('input_camera_info_topic').perform(context)
        output_depth_topic = LaunchConfiguration('output_depth_topic').perform(context)
        output_depth_info_topic = LaunchConfiguration('output_depth_info_topic').perform(context)
        output_corrected_image_topic = LaunchConfiguration('output_corrected_image_topic').perform(context)
        
        # Advanced parameters
        log_level = LaunchConfiguration('log_level').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        config_file = LaunchConfiguration('config_file').perform(context)
        
        # Parse input resolution
        try:
            input_res = eval(input_resolution)
            if not isinstance(input_res, list) or len(input_res) != 2:
                input_res = [384, 384]
        except:
            input_res = [384, 384]
        
        # Build parameters dictionary
        node_parameters = {
            # Original MiDaS parameters
            'model_type': model_type,
            'input_resolution': input_res,
            'camera_downsample_factor': camera_downsample_factor,
            'optimize_transforms': optimize_transforms,
            'use_half_precision': use_half_precision,
            'depth_scale_factor': depth_scale_factor,
            'max_depth': max_depth,
            'min_depth': min_depth,
            
            # Fisheye correction parameters
            'enable_fisheye_correction': enable_fisheye_correction,
            'fisheye_alpha': fisheye_alpha,
            'auto_extract_camera_params': auto_extract_camera_params,
            'crop_corrected_image': crop_corrected_image,
            'crop_ratio': crop_ratio,
            
            # Endoscopic preprocessing parameters
            'apply_endoscopic_preprocessing': apply_preprocessing,
            'clahe_clip_limit': clahe_clip_limit,
            'clahe_tile_grid_size': clahe_tile_grid_size,
            
            # Performance parameters
            'enable_performance_logging': enable_performance_logging,
            
            # ROS parameters
            'use_sim_time': use_sim_time,
        }
        
        # Add config file parameters if specified
        parameter_sources = [node_parameters]
        if config_file and os.path.exists(config_file):
            parameter_sources.append(config_file)
        
        # Create remapping for topics
        remappings = [
            (input_image_topic, input_image_topic),
            (input_camera_info_topic, input_camera_info_topic),
            (output_depth_topic, output_depth_topic),
            (output_depth_info_topic, output_depth_info_topic),
        ]
        
        if publish_corrected_image:
            remappings.append((output_corrected_image_topic, output_corrected_image_topic))
        
        # Create the node
        midas_node = Node(
            package=package_name,
            executable=executable_name,
            name=node_name,
            parameters=parameter_sources,
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
            emulate_tty=True,
        )
        
        return [midas_node]
    
    # ========================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ========================================
    
    return LaunchDescription([
        # Original MiDaS parameters
        declare_model_type,
        declare_input_resolution,
        declare_camera_downsample_factor,
        declare_optimize_transforms,
        declare_use_half_precision,
        declare_depth_scale_factor,
        declare_max_depth,
        declare_min_depth,
        
        # Fisheye correction parameters
        declare_enable_fisheye_correction,
        declare_fisheye_alpha,
        declare_auto_extract_camera_params,
        declare_crop_corrected_image,
        declare_crop_ratio,
        
        # Endoscopic preprocessing parameters
        declare_apply_endoscopic_preprocessing,
        declare_apply_vesica_preprocessing,  # Backward compatibility
        declare_clahe_clip_limit,
        declare_clahe_tile_grid_size,
        
        # Performance and debugging
        declare_enable_performance_logging,
        declare_publish_corrected_image,
        
        # Topic remapping
        declare_input_image_topic,
        declare_input_camera_info_topic,
        declare_output_depth_topic,
        declare_output_depth_info_topic,
        declare_output_corrected_image_topic,
        
        # Advanced configuration
        declare_log_level,
        declare_use_sim_time,
        declare_config_file,
        
        # Create the node with all parameters
        OpaqueFunction(function=create_node_with_parameters),
    ])


# ========================================
# USAGE EXAMPLES AND DOCUMENTATION
# ========================================

"""
USAGE EXAMPLES:

1. Basic launch with default parameters (recommended for most users):
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py

2. Launch with custom fisheye correction settings:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       fisheye_alpha:=0.8 crop_ratio:=0.9

3. Launch with performance optimization for RTX 3070:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       model_type:=dpt_hybrid_384 use_half_precision:=true

4. Launch with custom topic names for multi-camera setup:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       input_image_topic:=/camera1/image_raw \
       output_depth_topic:=/camera1/depth/image_raw

5. Launch with config file for complex setups:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       config_file:=/path/to/your/endoscope_config.yaml

6. Debug mode with detailed logging:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       log_level:=debug enable_performance_logging:=true

7. Simulation/rosbag replay mode:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       use_sim_time:=true

8. Backward compatibility with old vesica parameter:
   ros2 launch endoscope_depth_estimation midas_fisheye_depth.launch.py \
       apply_vesica_preprocessing:=true

PARAMETER RECOMMENDATIONS:

For Endoscopic UR3 Setup:
- model_type: dpt_hybrid_384 (best balance of speed/accuracy)
- fisheye_alpha: 0.7 (good compromise for endoscopes)
- crop_ratio: 0.85 (removes artifacts, keeps useful area)
- max_depth: 500.0 (good for tabletop objects)
- min_depth: 80.0 (prevents robot arm interference)

For High Performance (RTX 3070):
- use_half_precision: true
- optimize_transforms: true
- camera_downsample_factor: 1 (unless you need speed over quality)

For High Accuracy:
- model_type: dpt_large_384
- input_resolution: [512, 512]
- camera_downsample_factor: 1
"""

