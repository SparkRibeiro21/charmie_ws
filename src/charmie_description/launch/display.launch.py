from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path

from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files

    urdf_path = os.path.join(get_package_share_path('charmie_description'), 
                             'urdf', 'charmie_real.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('charmie_description'), 
                             'rviz', 'urdf_config.rviz')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    # not used in real robot (so had to be manually added)
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

    # not used in real robot (so had to be manually added)
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

    # not implemented yet, we assume everytime we need TFs the robot legs and torso are on top position
    static_transform_torso_legs = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_right_wheel',
        arguments=[
            '0',  # x
            '0',  # y
            '0.475',  # z
            '0',  # roll
            '0',  # pitch
            '0',  # yaw
            'base_link',  # parent frame
            'legs_link'  # child frame
        ]
    )

    # not implemented yet, we assume everytime we need TFs the robot legs and torso are on top position
    static_transform_torso_torso = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_right_wheel',
        arguments=[
            '0',  # x
            '0',  # y
            '0.25',  # z
            '0',  # roll
            '0.15708',  # pitch
            '0',  # yaw
            'legs_link',  # parent frame
            'torso_link'  # child frame
        ]
    )

    # not implemented yet, we assume everytime we need TFs the robot legs and torso are on top position
    static_transform_head_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_right_wheel',
        arguments=[
            '0',  # x
            '0',  # y
            '0',  # z
            '0',  # roll
            '0',  # pitch
            '0',  # yaw
            'head_camera_link',  # parent frame
            'D455_head_link'  # child frame
        ]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        static_transform_left_wheel,
        static_transform_right_wheel,
        static_transform_torso_legs,
        static_transform_torso_torso,
        static_transform_head_camera,
        rviz2_node
    ])