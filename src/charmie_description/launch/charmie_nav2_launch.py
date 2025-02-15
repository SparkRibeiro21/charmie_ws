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
                             'urdf', 'charmie_gazebo.urdf.xacro')
    
    rviz_config_path = os.path.join(get_package_share_path('charmie_description'), 
                             'rviz', 'nav2_config.rviz')
    
    # gazebo_ros_path = get_package_share_path('gazebo_ros')

    # world_path = os.path.join(get_package_share_path('charmie_description'), 
    #                          'worlds', 'obstacles_test.world')
    
    # twist_mux_config_path = os.path.join(get_package_share_path('charmie_description'), 
    #                          'config', 'twist_mux.yaml')

    # Some ROS 2 packages (especially when running inside containers or strict environments) might not resolve symlinks properly.?
    map_path = os.path.join(get_package_share_path('configuration_files'),
                            'maps', '2025_LAR_house_final_save.yaml')

    # nav2_bringup_path = get_package_share_path('nav2_bringup')
    nav2_bringup_path = get_package_share_path('charmie_description')
    
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

    navigation_with_ps4 = Node(package='charmie_demonstration',
                executable='navigation_demonstration',
                name='navigation_demonstration',
                emulate_tty=True
                )
    

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    # gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #             os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')),
    #             launch_arguments={'world': world_path}.items()  # Specify the world file     
    # )
    
    # gazebo = TimerAction(
    #     period=0.0,  # Delay localization to ensure Gazebo is ready
    #     actions=[
    #         IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #             os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')),
    #             launch_arguments={'world': world_path}.items()  # Specify the world file     
    #         )
    #     ]
    # )

    # gazebo_params_file = os.path.join(get_package_share_directory('charmie_description'),'config','gazebo_params.yaml')
              
    # gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #             os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')),
    #             launch_arguments={'extra_gazebo_args': '--ros-args --param-file ' + gazebo_params_file, 'world': world_path}.items()
    # )      

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    """
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'charmie'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    """
    # The differencial pluggin from gazebo does not receive messages from /cmd_vel but from /diff_cont/cmd_vel_unstamped
    # twist_mux_node = Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_config_path],
    #     remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    #     output="screen"
    # )
    
    # nav2_localization = TimerAction(
    #     period=5.0,  # Delay localization to ensure Gazebo is ready
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                os.path.join(nav2_bringup_path, 'launch', 'localization_launch.py')
    #             ),
    #             launch_arguments={'map': '/home/tiago/charmie_ws/src/configuration_files/maps/sim_tests/charmie_diff_sim_map_full_save.yaml', 'use_sim_time': 'true'}.items()
    #         )
    #     ]
    # )

    if not os.path.exists(map_path):
        print(f"ERROR: The map file does not exist: {map_path}")
    else:
        print(f"Map file found at: {map_path}")
        
    # Adds localization mode (AMCL) from nav2
    nav2_localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(
             os.path.join(nav2_bringup_path, 'launch', 'localization_launch.py')),
            launch_arguments=[
                # ('map', '/home/tiago/charmie_ws/src/configuration_files/maps/sim_tests/charmie_diff_sim_map_full_save.yaml'),
                ('use_sim_time', 'true')
                ]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        static_transform_left_wheel,
        static_transform_right_wheel,
        rviz2_node,


        # twist_mux_node,
        # spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        # gazebo,
        
        std_lf.odometry_lidar,
        # std_lf.gui,
        # std_lf.speakers,
        std_lf.lidar,
        std_lf.lidar_bottom,
        std_lf.low_level,
        # std_lf.ps4_controller,
        
        # nav2_localization,
        
    ])