from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node
from pathlib import Path

import os


def generate_launch_description():


    charmie_multi_camera_launch_file = PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'charmie_multi_camera_launch.py'
    )])

    # Use IncludeLaunchDescription to include the launch file
    charmie_multi_camera_launch_description = IncludeLaunchDescription(charmie_multi_camera_launch_file)


    inspection = Node(package='charmie_inspection',
                executable='inspection',
                name='inspection',
                emulate_tty=True
                )
    
    speakers = Node(package='charmie_speakers',
                        executable='speakers',
                        name='speakers',
                        emulate_tty=True
                        )
    
    neck = Node(package='charmie_neck_dynamixel',
                executable='neck_dynamixel',
                name='neck_dynamixel',
                parameters=[
                    {'device_name': "USB1"},
                ],
                emulate_tty=True
                )
    
    low_level = Node(package='charmie_low_level',
                executable='low_level',
                name='low_level',
                emulate_tty=True
                )
    
    face = Node(package='charmie_face',
                executable='face',
                name='face',
                parameters=[
                    {'show_speech': True},
                    {'after_speech_timer': 1.0},
                    {'initial_face': 'charmie_face'},
                    ],
                emulate_tty=True
                )
    
    yolo_pose = Node(package='charmie_yolo_pose',
                        executable='yolo_pose',
                        name='yolo_pose',
                        emulate_tty=True
                        )
    
    point_cloud = Node(package='charmie_point_cloud',
                        executable='point_cloud',
                        name='point_cloud',
                        emulate_tty=True
                        )
    
    lidar = Node(package='charmie_lidar_hokuyo',
                        executable='lidar_hokuyo',
                        name='lidar_hokuyo',
                        emulate_tty=True
                        )
    
    obstacles = Node(package='charmie_obstacles',
                        executable='obstacles_fusion',
                        name='obstacles_fusion',
                        emulate_tty=True
                        )
        
    odometry = Node(package='charmie_odometry',
                executable='odometry',
                name='odometry',
                emulate_tty=True
                )
    
    navigation = Node(package='charmie_navigation_sdnl',
                executable='navigation',
                name='navigation',
                emulate_tty=True
                )
    
    debug_visual = Node(package='charmie_debug',
                        executable='debug_visual',
                        name='debug_visual',
                        emulate_tty=True
                        )
    

    return LaunchDescription([
        charmie_multi_camera_launch_description,
        low_level,
        face,
        speakers,
        neck,
        odometry,
        # navigation,
        point_cloud,
        yolo_pose,
        lidar,
        obstacles,
        debug_visual,
        # inspection,
    ])
