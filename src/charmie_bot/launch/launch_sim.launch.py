import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit


def generate_launch_description():

    package_name='charmie_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

     # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'my_bot'],
                        output='screen')
    
    joint_state = Node(package='joint_state_publisher',
                       executable='joint_state_publisher',
                       name='joint_state_publisher',
                       output='screen',
                       parameters=[{'use_sim_time': True}])
    
    joint_state_broadcaster_spawner = Node(
                        package="controller_manager",
                        executable="spawner.py",
                        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],)
    
    joint_trajectory_controller_spawner = Node(
                        package="controller_manager",
                        executable="spawner.py",
                        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],)
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joint_state
    ])