from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='midas_depth_ros',
            executable='midas_depth_node',
            name='midas_depth_node',
            output='screen',
            parameters=[],
            remappings=[
                ('endoscope/image_raw', '/endoscope/image_raw'),
                ('endoscope/depth_image', '/endoscope/depth_image'),
            ]
        )
    ])
