from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path

from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files

    urdf_path = os.path.join(get_package_share_path('charmie_description'), 
                             'urdf', 'charmie_gazebo.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('charmie_description'), 
                             'rviz', 'urdf_config.rviz')
    
    gazebo_ros_path = get_package_share_path('gazebo_ros')

    world_path = os.path.join(get_package_share_path('charmie_description'), 
                             'worlds', 'obstacles_test.world')
    
    slam_mapper_params_path = os.path.join(get_package_share_path('charmie_description'), 'config', 'mapper_params_online_async.yaml')
    slam_toolbox_launch_file = os.path.join(get_package_share_path('slam_toolbox'), 'launch', 'online_async_launch.py')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')),
                launch_arguments={
                    'world': world_path
                }.items()  # Specify the world file
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'charmie'],
                        output='screen')

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'slam_params_file': slam_mapper_params_path,
            'use_sim_time': 'true'
        }.items()
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        rviz2_node,
        spawn_entity,
        gazebo,
        slam_toolbox_launch,

        std_lf.odometry_lidar
    ])