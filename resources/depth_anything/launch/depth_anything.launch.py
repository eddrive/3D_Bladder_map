#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for Depth-Anything V2 depth estimation node with fisheye correction"""
    
    # === ARGOMENTI CORE DEPTH-ANYTHING V2 ===
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='vitl',  # vitl (migliore), vitb, vits (veloce)
        description='Depth-Anything V2 model type: vitl=Large, vitb=Base, vits=Small'
    )
    
    input_resolution_arg = DeclareLaunchArgument(
        'input_resolution',
        default_value='[518, 518]',  # Ottimale per Depth-Anything V2
        description='Input resolution for Depth-Anything V2 model'
    )
    
    camera_downsample_arg = DeclareLaunchArgument(
        'camera_downsample_factor',
        default_value='1',  # No downsample per qualità vescica
        description='Downsample factor for input (1=no downsample, 2=half size)'
    )
    
    # === ARGOMENTI PERFORMANCE ===
    use_half_precision_arg = DeclareLaunchArgument(
        'use_half_precision',
        default_value='true',
        description='Use FP16 for faster inference on RTX 3070'
    )
    
    optimize_transforms_arg = DeclareLaunchArgument(
        'optimize_transforms',
        default_value='true',
        description='Optimize transforms for speed'
    )
    
    # === ARGOMENTI VESCICA ===
    vesica_preprocessing_arg = DeclareLaunchArgument(
        'apply_vesica_preprocessing',
        default_value='true',
        description='Apply preprocessing optimized for bladder endoscopy (CLAHE, specular reduction)'
    )
    
    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='150.0',  # 15cm realistico per vescica
        description='Maximum expected depth in mm (vescica range)'
    )
    
    min_depth_arg = DeclareLaunchArgument(
        'min_depth',
        default_value='10.0',  # 1cm minimo vescica
        description='Minimum expected depth in mm (vescica range)'
    )
    
    depth_scale_factor_arg = DeclareLaunchArgument(
        'depth_scale_factor',
        default_value='1.0',  # Depth-Anything V2 output già corretto
        description='Scale factor for depth values (1.0 for Depth-Anything V2)'
    )
    
    # === ARGOMENTI MASCHERA ===
    endoscope_mask_arg = DeclareLaunchArgument(
        'endoscope_mask_path',
        default_value='/root/endoscope_mask.png',
        description='Path to endoscope ROI mask file'
    )
    
    # === ARGOMENTI SISTEMA ===
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if running with simulation'
    )
    
    # === ARGOMENTI DEBUG ===
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug output and depth visualization'
    )
    
    
    # === NODO DEPTH-ANYTHING V2 ===
    depth_anything_v2_node = Node(
        package='depth_anything',
        executable='depth_anything.py',
        name='depth_anything_v2_endoscope_node',
        output='screen',
        parameters=[{
            # Core Depth-Anything V2 parameters
            'model_type': LaunchConfiguration('model_type'),
            'input_resolution': LaunchConfiguration('input_resolution'),
            'camera_downsample_factor': LaunchConfiguration('camera_downsample_factor'),
            'use_half_precision': LaunchConfiguration('use_half_precision'),
            'optimize_transforms': LaunchConfiguration('optimize_transforms'),
            
            # Vescica-specific parameters
            'apply_vesica_preprocessing': LaunchConfiguration('apply_vesica_preprocessing'),
            'max_depth': LaunchConfiguration('max_depth'),
            'min_depth': LaunchConfiguration('min_depth'),
            'depth_scale_factor': LaunchConfiguration('depth_scale_factor'),
            
            # Mask parameters
            'endoscope_mask_path': LaunchConfiguration('endoscope_mask_path'),
            
            # System parameters
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    
    return LaunchDescription([
        # === LAUNCH ARGUMENTS ===
        model_type_arg,
        input_resolution_arg,
        camera_downsample_arg,
        use_half_precision_arg,
        optimize_transforms_arg,
        vesica_preprocessing_arg,
        max_depth_arg,
        min_depth_arg,
        depth_scale_factor_arg,
        endoscope_mask_arg,
        use_sim_time_arg,
        debug_mode_arg,
        
        # === NODES ===
        depth_anything_v2_node,
        # debug_visualizer_node,  # Solo se debug_mode=true
    ])