from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('charmie_description'), 
                             'urdf', 'charmie_gazebo.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('charmie_description'), 
                             'rviz', 'urdf_config.rviz')
    
    slam_mapper_params_path = os.path.join(get_package_share_path('charmie_description'), 'config', 'mapper_params_online_async.yaml')
    slam_toolbox_launch_file = os.path.join(get_package_share_path('slam_toolbox'), 'launch', 'online_async_launch.py')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'slam_params_file': slam_mapper_params_path,
            'use_sim_time': 'false'
        }.items()
    )

    static_transform_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_left_wheel',
        arguments=[
            '0',  # x
            '0.255',  # y
            '0.05',  # z
            '0',  # roll
            '0',  # pitch
            '0',  # yaw
            'base_link',  # parent frame
            'left_wheel_link'  # child frame
        ]
    )

    static_transform_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_right_wheel',
        arguments=[
            '0',  # x
            '-0.255',  # y
            '0.05',  # z
            '0',  # roll
            '0',  # pitch
            '0',  # yaw
            'base_link',  # parent frame
            'right_wheel_link'  # child frame
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        static_transform_left_wheel,
        static_transform_right_wheel,
        rviz2_node,
        slam_toolbox_launch
    ])