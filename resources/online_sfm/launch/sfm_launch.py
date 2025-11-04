#!/usr/bin/env python3

"""
Launch file per SfM Node
Carica configurazione da sfm_params.yaml e avvia il nodo SfM
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Ottieni il percorso del package
    package_name = 'online_sfm'  # Nome package corretto
    
    # Dichiarazione degli argomenti di launch
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'sfm_params.yaml'
        ]),
        description='Path al file di configurazione SfM'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Livello di logging (debug, info, warn, error, fatal)'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Abilita logging diagnostico dettagliato'
    )
    
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='world',
        description='Frame di riferimento mondiale'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', 
        default_value='camera',
        description='Frame camera (endoscopio TF: Z out, X right, Y down)'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/endoscope/image_raw',
        description='Topic immagine raw'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/endoscope/camera_info',
        description='Topic camera info'
    )
    
    # Nodo SfM principale
    sfm_node = Node(
        package=package_name,
        executable='sfm_node',
        name='sfm_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'publishing.world_frame': LaunchConfiguration('world_frame'),
                'publishing.camera_frame': LaunchConfiguration('camera_frame'),
                'diagnostics.enable_detailed_logging': LaunchConfiguration('enable_diagnostics'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Rimappa i topic se necessario
            ('/endoscope/image_raw', LaunchConfiguration('image_topic')),
            ('/endoscope/camera_info', LaunchConfiguration('camera_info_topic')),
        ]
    )
    
    # Log informativo
    log_info = LogInfo(
        msg=[
            '\n',
            '🔬 SfM NODE LAUNCH STARTED\n',
            '   📹 Camera Frame: ', LaunchConfiguration('camera_frame'), '\n',
            '   🌍 World Frame: ', LaunchConfiguration('world_frame'), '\n', 
            '   ⚙️ Config File: ', LaunchConfiguration('config_file'), '\n',
            '   📊 Log Level: ', LaunchConfiguration('log_level'), '\n',
            '   🔍 Diagnostics: ', LaunchConfiguration('enable_diagnostics'), '\n',
            '   📡 Input Topics:\n',
            '      - Image: ', LaunchConfiguration('image_topic'), '\n',
            '      - Camera Info: ', LaunchConfiguration('camera_info_topic'), '\n',
            '   📡 Output Topics:\n',
            '      - Point Cloud: /sfm/point_cloud\n',
            '      - Trajectory: /sfm/trajectory\n',
            '   🎯 Ottimizzato per: 1080p endoscope, oggetti 20cm, TF(Z:out, X:right, Y:down)\n'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        log_level_arg,
        enable_diagnostics_arg,
        world_frame_arg,
        camera_frame_arg,
        image_topic_arg,
        camera_info_topic_arg,
        
        # Log info
        log_info,
        
        # Main SfM node
        sfm_node,
    ])


# Esempi di lancio da linea di comando:
#
# 1. Lancio standard:
# ros2 launch online_sfm sfm_launch.py
#
# 2. Con config personalizzato:
# ros2 launch online_sfm sfm_launch.py config_file:=/path/to/custom_config.yaml
#
# 3. Con debug abilitato:
# ros2 launch online_sfm sfm_launch.py log_level:=debug enable_diagnostics:=true
#
# 4. Con frame personalizzati:
# ros2 launch online_sfm sfm_launch.py world_frame:=odom camera_frame:=endoscope_optical_frame
#
# 5. Con topic personalizzati:
# ros2 launch online_sfm sfm_launch.py image_topic:=/camera/image_raw camera_info_topic:=/camera/camera_info
#
# 6. Combinazione completa:
# ros2 launch online_sfm sfm_launch.py \
#   config_file:=/path/custom.yaml \
#   log_level:=debug \
#   world_frame:=map \
#   camera_frame:=endoscope_link \
#   image_topic:=/custom/image \
#   camera_info_topic:=/custom/camera_info