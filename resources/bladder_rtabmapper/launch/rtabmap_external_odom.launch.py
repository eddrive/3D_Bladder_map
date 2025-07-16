from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('bladder_rtabmapper'),
                'params',
                'rtabmap_external_odom.yaml'
            ]),
            description='Full path to the RTAB-Map parameters YAML file'),

        DeclareLaunchArgument('rgb_topic',
            default_value='/endoscope/image_raw',
            description='RGB image topic'),

        DeclareLaunchArgument('camera_info_topic',
            default_value='/endoscope/camera_info',
            description='Camera info topic'),

        DeclareLaunchArgument('odom_topic',
            default_value='/odom',
            description='Odometry topic'),

        DeclareLaunchArgument('frame_id',
            default_value='base_link',
            description='Fixed frame id for RTAB-Map'),

        DeclareLaunchArgument('parent_frame',
            default_value='base_link',
            description='TF parent frame (for odom publisher)'),

        DeclareLaunchArgument('child_frame',
            default_value='camera',
            description='TF child frame (for odom publisher)'),
        # 1. Frame map statico (punto di inizio mappatura)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # 2. Frame odom -> base_link (radice del manipolatore)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'world'],
            output='screen'
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file')
            ],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('odom', LaunchConfiguration('odom_topic'))
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