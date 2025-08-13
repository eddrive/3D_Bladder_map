from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='online_sfm',
            executable='sfm_node',
            name='endoscope_sfm',
            output='screen',
            parameters=[{
                # Parametri per oggetto 20cm
                'min_baseline_meters': 0.003,      # 3mm
                'min_rotation_degrees': 0.8,       # 0.8°
                'ransac_threshold': 12.0,          # Fisheye
            }],
            # I topic sono GIÀ CORRETTI - non serve remapping!
            # Le tue subscription sono già su:
            # /endoscope/image_raw ✅
            # /endoscope/camera_info ✅  
        ),
    ])