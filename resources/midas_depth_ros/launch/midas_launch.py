from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for MiDaS depth estimation node with fisheye correction"""
    
    # Declare launch arguments
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='dpt_hybrid_384',
        description='MiDaS model type'
    )
    
    input_resolution_arg = DeclareLaunchArgument(
        'input_resolution',
        default_value='[384, 384]',
        description='Input resolution for MiDaS model'
    )
    
    camera_downsample_arg = DeclareLaunchArgument(
        'camera_downsample_factor',
        default_value='2',
        description='Downsample factor for 1280x720 input (2 = 640x360)'
    )
    
    use_half_precision_arg = DeclareLaunchArgument(
        'use_half_precision',
        default_value='true',
        description='Use FP16 for faster inference on RTX 3070'
    )
    
    vesica_preprocessing_arg = DeclareLaunchArgument(
        'apply_vesica_preprocessing',
        default_value='true',
        description='Apply preprocessing optimized for bladder endoscopy'
    )
    
    depth_scale_factor_arg = DeclareLaunchArgument(
        'depth_scale_factor',
        default_value='1000.0',
        description='Scale factor for depth values (1000 = mm)'
    )
    
    # AGGIORNATO: Range più realistico per endoscopi
    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='2000.0',  # 2m invece di 200mm
        description='Maximum expected depth in mm (endoscope range)'
    )
    
    # NUOVO: Parametro per maschera endoscopio
    endoscope_mask_arg = DeclareLaunchArgument(
        'endoscope_mask_path',
        default_value='/root/endoscope_mask.png',
        description='Path to endoscope ROI mask file'
    )
    
    # OPZIONALE: Parametri aggiuntivi per debug
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug output and visualization'
    )
    
    # MiDaS depth estimation node with fisheye correction
    midas_node = Node(
        package='midas_depth_ros',  # Assicurati che questo sia il nome corretto del tuo package
        executable='midas_depth_node.py',
        name='midas_depth_node',
        output='screen',
        parameters=[{
            'model_type': LaunchConfiguration('model_type'),
            'input_resolution': LaunchConfiguration('input_resolution'),
            'camera_downsample_factor': LaunchConfiguration('camera_downsample_factor'),  
            'use_half_precision': LaunchConfiguration('use_half_precision'),
            'apply_vesica_preprocessing': LaunchConfiguration('apply_vesica_preprocessing'),
            'depth_scale_factor': LaunchConfiguration('depth_scale_factor'),
            'max_depth': LaunchConfiguration('max_depth'),
            'endoscope_mask_path': LaunchConfiguration('endoscope_mask_path'),  # NUOVO
            'optimize_transforms': True,
        }],
        # OPZIONALE: Remapping se i tuoi topic hanno nomi diversi
        remappings=[
            # ('/endoscope/image_raw', '/your_camera/image_raw'),
            # ('/endoscope/camera_info', '/your_camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        model_type_arg,
        input_resolution_arg,
        camera_downsample_arg,  
        use_half_precision_arg,
        vesica_preprocessing_arg,
        depth_scale_factor_arg,
        max_depth_arg,
        endoscope_mask_arg,  # NUOVO
        debug_mode_arg,      # OPZIONALE
        midas_node,
    ])