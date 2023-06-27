import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import Command

from launch_ros.actions import Node

from launch.event_handlers import OnProcessStart

import xacro

def generate_launch_description():

    package_name='charmie_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name), "config", "controller_config.yaml")
    rviz2_file = os.path.join(get_package_share_directory(package_name), "config", "view_bot.rviz")
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
        
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    delayed_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_trajectory_controller_spawner],
        )
    )

    rviz2_spawner = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz2_file]
    )

    lidar_setup = Node(
        package="charmie_lidar_hokuyo",
        executable="lidar_hokuyo",
        name="lidar_hokuyo",
    )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_joint_state_broadcaster_spawner,
        delayed_joint_trajectory_controller_spawner,
        rviz2_spawner
    ])