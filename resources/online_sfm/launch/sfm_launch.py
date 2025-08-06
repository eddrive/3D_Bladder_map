from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='online_sfm',
            executable='sfm_node',
            name='sfm_node',
            output='screen',
            parameters=[],
        )
    ])