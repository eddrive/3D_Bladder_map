from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    config_file_path = PathJoinSubstitution([
        FindPackageShare('online_sfm'),
        'config',
        'sfm_params.yaml'
    ])
    
    return LaunchDescription([
        # Declare launch arguments with default values
        DeclareLaunchArgument(
            'use_config_file',
            default_value='true',
            description='Whether to use the config file or launch arguments'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Path to the configuration file'
        ),
        DeclareLaunchArgument(
            'min_point_distance',
            default_value='0.05',
            description='Minimum distance between points to avoid duplicates'
        ),
        DeclareLaunchArgument(
            'min_observations', 
            default_value='2',
            description='Minimum number of observations to keep a point'
        ),
        DeclareLaunchArgument(
            'max_points',
            default_value='10000', 
            description='Maximum number of points to maintain in the map'
        ),
        DeclareLaunchArgument(
            'publish_every_n_frames',
            default_value='5',
            description='Publish accumulated point cloud every N frames'
        ),
        DeclareLaunchArgument(
            'ransac_threshold',
            default_value='3.0',
            description='RANSAC reprojection error threshold in pixels'
        ),
        DeclareLaunchArgument(
            'ransac_min_inliers',
            default_value='20',
            description='Minimum number of RANSAC inliers required'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/endoscope/image_raw',
            description='Input image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', 
            default_value='/endoscope/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='world',
            description='World/reference frame name'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera',
            description='Camera frame name'
        ),
        
        # SfM Node
        Node(
            package='online_sfm',
            executable='sfm_node',
            name='online_sfm_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    # These will override config file values if use_config_file is false
                    'min_point_distance': LaunchConfiguration('min_point_distance'),
                    'min_observations': LaunchConfiguration('min_observations'), 
                    'max_points': LaunchConfiguration('max_points'),
                    'publish_every_n_frames': LaunchConfiguration('publish_every_n_frames'),
                    'ransac_threshold': LaunchConfiguration('ransac_threshold'),
                    'ransac_min_inliers': LaunchConfiguration('ransac_min_inliers'),
                    'world_frame': LaunchConfiguration('world_frame'),
                    'camera_frame': LaunchConfiguration('camera_frame'),
                }
            ],
            remappings=[
                ('/endoscope/image_raw', LaunchConfiguration('image_topic')),
                ('/endoscope/camera_info', LaunchConfiguration('camera_info_topic')),
            ]
        ),
    ])