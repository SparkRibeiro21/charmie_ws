from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node
from pathlib import Path

import os


def generate_launch_description():

    double_cameras_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch/charmie_multi_camera_launch.py')
        )
    )

    point_cloud = Node(package='charmie_point_cloud',
                        executable='point_cloud',
                        name='point_cloud',
                        emulate_tty=True
                        )

    yolo_pose = Node(package='charmie_yolo_pose',
                        executable='yolo_pose',
                        name='yolo_pose',
                        parameters=[
                            {'yolo_model': "s"},
                            {'debug_draw': True},
                            {'characteristics': True},
                            ],
                        emulate_tty=True
                        )
    
    yolo_objects = Node(package='charmie_yolo_objects',
                        executable='yolo_objects',
                        name='yolo_objects',
                        emulate_tty=True
                        )
    
    debug_yolos = Node(package='charmie_debug',
                        executable='debug_yolos',
                        name='debug_yolos',
                        emulate_tty=True
                        )
    
    
    return LaunchDescription([
        double_cameras_launch_file,
        point_cloud,
        yolo_pose,
        # yolo_objects,
        debug_yolos,
    ])
