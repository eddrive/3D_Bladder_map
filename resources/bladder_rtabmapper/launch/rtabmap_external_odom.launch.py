from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('parent_frame',
            default_value='base_link',
            description='TF parent frame (for odom publisher)'),
        
        DeclareLaunchArgument('child_frame',
            default_value='camera',
            description='TF child frame (for odom publisher)'),
        
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('bladder_rtabmapper'),
                'params',
                'rtabmap_external_odom.yaml'
            ]),
            description='Full path to the RTAB-Map parameters YAML file'),
        
        # RTABMap node - tutti i parametri dal YAML
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file')
            ],
            remappings=[
                ('rgb/image', '/endoscope/image_corrected'),
                ('rgb/camera_info', '/endoscope/camera_info_corrected'),
                ('depth/image', '/endoscope/depth_corrected/image_raw'),
                ('depth/camera_info', '/endoscope/depth_corrected/camera_info'),
                ('odom', '/odom')
            ]
        ),
        
        Node(
            package='bladder_rtabmapper',
            executable='tf_to_odom_publisher',
            name='tf_to_odom_publisher',
            output='screen',
            parameters=[{
                'parent_frame': LaunchConfiguration('parent_frame'),
                'child_frame': LaunchConfiguration('child_frame')
            }]
        )
    ])