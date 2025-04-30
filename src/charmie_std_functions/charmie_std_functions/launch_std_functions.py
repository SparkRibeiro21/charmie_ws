from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch.substitutions import Command
from launch_ros.actions import Node
from pathlib import Path

import xacro
from datetime import datetime
import os

class LaunchStdFunctions():

    def __init__(self):

        ### ROBOT URDF 
        urdf_path_real = os.path.join(get_package_share_path('charmie_description'), 
                             'urdf', 'charmie_real.urdf.xacro')
        robot_description_real = ParameterValue(Command(['xacro ', urdf_path_real]), value_type=str)

        self.robot_state_publisher_real_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description_real}]
        )

        urdf_path_gazebo = os.path.join(get_package_share_path('charmie_description'), 
                             'urdf', 'charmie_gazebo.urdf.xacro')
        robot_description_gazebo = ParameterValue(Command(['xacro ', urdf_path_gazebo]), value_type=str)

        self.robot_state_publisher_gazebo_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description_gazebo}]
        )


        ### RVIZ CONFIGS
        rviz_basic_config_path = os.path.join(get_package_share_path('charmie_description'), 
                                'rviz', 'urdf_config.rviz')
        
        self.rviz2_basic_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_basic_config_path]
        )

        rviz_slam_config_path = os.path.join(get_package_share_path('charmie_description'), 
                                'rviz', 'slam_config.rviz')
    
        self.rviz2_slam_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_slam_config_path]
        )

        rviz_nav2_config_path = os.path.join(get_package_share_path('charmie_description'), 
                                'rviz', 'nav2_config.rviz')
        
        self.rviz2_nav2_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_nav2_config_path]
        )

        ### GAZEBO
        gazebo_ros_path = get_package_share_path('gazebo_ros')

        world_path = os.path.join(get_package_share_path('charmie_description'), 
                                'worlds', 'obstacles_test.world')
        
        self.gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')),
                    launch_arguments={'world': world_path}.items()  # Specify the world file
        )

        # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
        self.spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-topic', 'robot_description',
                                    '-entity', 'charmie'],
                            output='screen')
        
        twist_mux_config_path = os.path.join(get_package_share_path('charmie_description'), 
                                'config', 'twist_mux.yaml')
        
        self.diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
        )

        self.joint_broad_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
        )

        # The differencial pluggin from gazebo does not receive messages from /cmd_vel but from /diff_cont/cmd_vel_unstamped
        self.twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_config_path],
            remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
            output="screen"
        )
        

        ### ARM XARM
        # Declare arguments
        self.declared_arm_arguments = []
        self.declared_arm_arguments.append(
            DeclareLaunchArgument(
                'robot_ip',
                default_value='192.168.1.219',
                description='IP address by which the robot can be reached.',
            )
        )
        self.declared_arm_arguments.append(
            DeclareLaunchArgument(
                'report_type',
                default_value='normal',
                description='Tcp report type, default is normal, normal/rich/dev optional.',
            )
        )
        self.declared_arm_arguments.append(
            DeclareLaunchArgument(
                'hw_ns',
                default_value='xarm',
                description='The namespace of xarm_driver, default is xarm.',
            )
        )

        # Initialize Arguments
        robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.219')
        report_type = LaunchConfiguration('report_type', default='normal')
        hw_ns = LaunchConfiguration('hw_ns', default='xarm')
        add_gripper = LaunchConfiguration('add_gripper', default=True)
        add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
        show_rviz = LaunchConfiguration('show_rviz', default=False)


        home = str(Path.home())

        # robot driver launch
        # xarm_api/launch/_robot_driver.launch.py
        self.robot_arm_driver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(home + '/charmie_ws/src/charmie_arm_ufactory/xarm_ros2/xarm_api/launch/_robot_driver.launch.py'),
            launch_arguments={
                'robot_ip': robot_ip,
                'report_type': report_type,
                'dof': '6',
                'hw_ns': hw_ns,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'show_rviz': show_rviz,
                'robot_type': 'xarm',
            }.items(),
        )

        #Publishes the joint_states of the robot
        self.joint_state_publisher = Node(package='joint_state_publisher',
                       executable='joint_state_publisher',
                       name='joint_state_publisher',
                       parameters=[{
                            'zeros': {
                                'xarm_joint1': -3.926,
                                'xarm_joint2': 1.449,
                                'xarm_joint3': -1.134,
                                'xarm_joint4': -0.017,
                                'xarm_joint5': 1.309,
                                'xarm_joint6': 4.712
                            }
                        }]
        )

        self.joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                       executable='joint_state_publisher_gui',
                       parameters=[{
                            'zeros': {
                                'xarm_gripper_drive_joint': 0.850,
                                'xarm_joint1': -3.926,
                                'xarm_joint2': 1.449,
                                'xarm_joint3': -1.134,
                                'xarm_joint4': -0.017,
                                'xarm_joint5': 1.309,
                                'xarm_joint6': 4.712
                            }
                        }]
        )
        
        #Test joint values
        #                        'xarm_gripper_drive_joint': 0.850,
        #                        'xarm_joint1': -2.988,
        #                        'xarm_joint2': 0.776672,
        #                        'xarm_joint3': -1.05767,
        #                        'xarm_joint4': 0.150098,
        #                        'xarm_joint5': 1.88496,
        #                        'xarm_joint6': 4.75951


        # Use IncludeLaunchDescription to include the launch file
        self.charmie_multi_camera_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'charmie_multi_camera_launch.py')])
            )

            # Include static transforms
        self.static_transforms_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('charmie_description'), 'launch', 'static_transforms_launch.py')])
            )
        
        # Use IncludeLaunchDescription to include the launch file
        self.charmie_orbbec_base_camera_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'astra.launch.py')])
            )

        self.gui = Node(package='charmie_gui',
                executable='gui_debug',
                name='gui_debug',
                )
        
        self.audio = Node(package='charmie_audio',
                      executable='audio',
                      name = 'audio',
                      )
        
        self.speakers = Node(package='charmie_speakers',
                            executable='speakers',
                            name='speakers',
                            emulate_tty=True
                            )
        
        self.neck = Node(package='charmie_neck_dynamixel',
                    executable='neck_dynamixel',
                    name='neck_dynamixel',
                    emulate_tty=True
                    )
        
        self.low_level = Node(package='charmie_low_level',
                    executable='low_level',
                    name='low_level',
                    emulate_tty=True
                    )
        
        self.face = Node(package='charmie_face',
                    executable='face',
                    name='face',
                    parameters=[
                        {'show_speech': True},
                        {'after_speech_timer': 1.0},
                        {'initial_face': 'charmie_face'},
                        ],
                    emulate_tty=True
                    )

        self.arm = Node(package='charmie_arm_ufactory',
                            executable='arm',
                            name='arm',
                            emulate_tty=True
                            )
        
        self.yolo_objects = Node(package='charmie_yolo_objects',
                            executable='yolo_objects',
                            name='yolo_objects',
                            emulate_tty=True
                            )
        
        self.yolo_pose = Node(package='charmie_yolo_pose',
                            executable='yolo_pose',
                            name='yolo_pose',
                            emulate_tty=True
                            )
        
        self.point_cloud = Node(package='charmie_point_cloud',
                            executable='point_cloud',
                            name='point_cloud',
                            emulate_tty=True
                            )
        
        self.lidar = Node(package='charmie_lidar_hokuyo',
                            executable='lidar_hokuyo',
                            name='lidar_hokuyo',
                            emulate_tty=True
                            )
        
        self.lidar_bottom = Node(package='charmie_lidar_hokuyo',
                            executable='lidar_hokuyo_bottom',
                            name='lidar_hokuyo_bottom',
                            emulate_tty=True
                            )
        
        self.obstacles = Node(package='charmie_obstacles',
                            executable='obstacles_fusion',
                            name='obstacles_fusion',
                            emulate_tty=True
                            )
        
        self.odometry = Node(package='charmie_odometry',
                    executable='odometry',
                    name='odometry',
                    emulate_tty=True
                    )
        
        self.odometry_lidar = Node(package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
                )
        
        self.navigation = Node(package='charmie_navigation_sdnl',
                    executable='navigation',
                    name='navigation',
                    emulate_tty=True
                    )
        
        self.ps4_controller = Node(package='charmie_ps4_controller',
                    executable='ps4_controller',
                    name='ps4_controller',
                    )
        
        self.navigation_with_ps4 = Node(package='charmie_demonstration',
                executable='navigation_demonstration',
                name='navigation_demonstration',
                emulate_tty=True
                )
    
        
        self.llm = Node(package='charmie_llm',
                    executable='llm',
                    name='llm',
                    )
        
        ### SLAM
        slam_mapper_params_path = os.path.join(get_package_share_path('charmie_description'), 'config', 'mapper_params_online_async.yaml')
        slam_toolbox_launch_file = os.path.join(get_package_share_path('slam_toolbox'), 'launch', 'online_async_launch.py')

        self.slam_toolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_file),
            launch_arguments={
                'slam_params_file': slam_mapper_params_path,
                'use_sim_time': 'false'
            }.items()
        )

        
        ### LOCALIZATION

        # Exmaples of how the map should be added to launch file
        map_path = os.path.join(get_package_share_path('configuration_files'), 'maps', '2025_LAR_house_final_save.yaml')
        # map_path = os.path.join(get_package_share_path('configuration_files'), 'maps', 'Robocup2023', 'robocup23_house_save.yaml')

        # Adds localization mode (AMCL) from nav2
        self.nav2_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_path('charmie_description'), 'launch', 'localization_launch.py')]),
                launch_arguments=[
                    ('map', map_path),
                    ('use_sim_time', 'false')
                    ]
        )

        self.delayed_nav2_localization = TimerAction(
            period=3.0,  # Delay for 3 seconds
            actions=[self.nav2_localization]  # This is the action that will be triggered after the delay
        )

        self.nav2_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_path('charmie_description'), 'launch', 'navigation_launch.py')]),
                launch_arguments=[
                    ('use_sim_time', 'false')
                    ]
        )
        
        ### ROSBAG
        # Node to start rosbag recording
        self.rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],  # Command to start recording all topics
        name='rosbag_record_node',
        output='screen'
        )
