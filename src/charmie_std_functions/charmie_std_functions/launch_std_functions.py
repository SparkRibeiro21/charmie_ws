from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, TextSubstitution

from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import Command, FindExecutable
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

        moveit_robot_description = Command([
            FindExecutable(name='xacro'), ' ',
            urdf_path_real, ' ',
            'use_real_hardware:=', LaunchConfiguration('use_real_hardware', default='true')
        ])

        self.robot_state_publisher_real_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': robot_description_real}],
            remappings=[('/joint_states','/xarm/joint_states')] # Comment this line if you just want to use charmie.display
        )

        self.robot_state_publisher_real_node_without_remappings = Node(
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
                                'rviz', 'nav2_default_view_charmie.rviz')
        
        self.rviz2_nav2_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_nav2_config_path]
        )

        rviz_calib_map_furniture_navigations_config_path = os.path.join(get_package_share_path('charmie_description'), 
                                'rviz', 'nav2_calibrate_maps_furniture_navigations.rviz')
        
        self.rviz2_calib_map_furniture_navigations_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_calib_map_furniture_navigations_config_path]
        )

        moveit_config = (MoveItConfigsBuilder("charmie", package_name="charmie_moveit_config")
                         .planning_pipelines("ompl")
                         .to_moveit_configs())

        moveit_rviz_path = os.path.join(get_package_share_path('charmie_description'), 
                                'rviz', 'moveit.rviz')

        self.moveit_rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', moveit_rviz_path],
            parameters=[
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ],
            output='screen'
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

        # added by Pedro to initialize the ROS2 control system for the xArm
        # ros2 control launch
        # xarm_controller/launch/_ros2_control.launch.py
        self.xarm_ros2_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(home + '/charmie_ws/src/charmie_arm_ufactory/xarm_ros2/xarm_controller/launch/_ros2_control.launch.py'),
            launch_arguments={
                'robot_ip': robot_ip,
                'report_type': report_type,
                'prefix': '',
                'hw_ns': hw_ns,
                'limited': 'false',
                'effort_control': 'false',
                'velocity_control': 'false',
                'add_gripper': add_gripper,
                'add_vacuum_gripper': 'false',
                'add_bio_gripper': 'false',
                'dof': '6',
                'robot_type': 'xarm',
                'ros2_control_plugin': 'uf_robot_hardware/UFRobotSystemHardware',
                'baud_checkset': 'true',
                'default_gripper_baud': '2000000',
                'use_sim_time': 'false'
            }.items()
        )

        ### MOVEIT
        self.moveit_commander = Node(
            package='charmie_commander_cpp',
            executable='moveit_commander',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ],
        )

        ros2_controllers_path = os.path.join(get_package_share_path('charmie_moveit_config'), 'config', 'ros2_controllers.yaml')

        self.ros2_control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': moveit_robot_description},
                ros2_controllers_path],
            output='screen'
        )

        self.xarm6_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['xarm6_controller'],
            output='screen'
        )

        self.joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )

        self.move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_path('charmie_moveit_config'),
                    'launch',
                    'move_group.launch.py')))

        #parameters added by Pedro
        #Publishes the joint_states of the robot
        self.joint_state_publisher = Node(package='joint_state_publisher',
                       executable='joint_state_publisher',
                       name='joint_state_publisher',
                       parameters=[{
                            'source_list': ['xarm/joint_states'],
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

        #parameters added by Pedro
        self.joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                       executable='joint_state_publisher_gui',
                       parameters=[{
                            'source_list': ['xarm/joint_states'],
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

        # Use IncludeLaunchDescription to include the launch file
        self.charmie_multi_camera_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'charmie_multi_camera_launch.py')])
            )

        # Include static transforms
        self.static_transforms_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('charmie_description'), 'launch', 'static_transforms_launch.py')])
            )
        
        # Include static transforms for localization using just odom (mainly for unmapped environments)
        self.static_transforms_just_odom_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('charmie_description'), 'launch', 'static_transforms_just_odom_launch.py')])
            )
        
        # Use IncludeLaunchDescription to include the launch file
        self.charmie_orbbec_base_camera_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'astra.launch.py')])
            )

        self.gui = Node(package='charmie_gui',
                executable='gui_debug',
                name='gui_debug',
                )

        self.marker_arrays_debug = Node(package='charmie_gui',
                executable='test_marker_rviz',
                name='test_marker_rviz',
                )
        
        self.audio = Node(package='charmie_audio',
                      executable='audio',
                      name = 'audio',
                      )
        
        self.sound_classification = Node(package='charmie_sound_classification',
                                        executable='sound_classification',
                                        name = 'sound_classification',
                                        )
        
        self.speakers = Node(package='charmie_speakers',
                            executable='speakers',
                            name='speakers',
                            emulate_tty=True
                            )
        
        self.speakers_with_save = Node(package='charmie_speakers',
                            executable='speakers_with_save',
                            name='speakers_with_save',
                            emulate_tty=True
                            )
        
        self.neck = Node(package='charmie_neck_dynamixel',
                    executable='neck_dynamixel',
                    name='neck_dynamixel',
                    emulate_tty=True
                    )
        
        self.low_level = Node(package='charmie_low_level',
                    executable='low_level_stream',
                    name='low_level_stream',
                    emulate_tty=True
                    )
        
        self.charmie_localisation = Node(package='charmie_localisation',
                    executable='localisation',
                    name='localisation',
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
        
        self.tracking = Node(package='charmie_tracking_sam2',
                            executable='tracking_sam2',
                            name='tracking_sam2',
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
        '''
        ################### user configure livox parameters for ros2 start ###################
        xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
        multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
        data_src      = 0    # 0-lidar, others-Invalid data src
        publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
        output_type   = 0
        frame_id      = 'livox_frame'
        lvx_file_path = '/home/livox/livox_test.lvx'
        cmdline_bd_code = 'livox0000000001'

        # cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
        # cur_config_path = cur_path + '../config'
        livox_pkg_path = os.path.join(get_package_share_path('livox_ros_driver2'))
        cur_config_path = livox_pkg_path + '/config'
        rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
        user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
        ################### user configure livox parameters for ros2 end #####################

        livox_ros2_params = [
            {"xfer_format": xfer_format},
            {"multi_topic": multi_topic},
            {"data_src": data_src},
            {"publish_freq": publish_freq},
            {"output_data_type": output_type},
            {"frame_id": frame_id},
            {"lvx_file_path": lvx_file_path},
            {"user_config_path": user_config_path},
            {"cmdline_input_bd_code": cmdline_bd_code}
        ]

        self.livox_driver = Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=livox_ros2_params
            )

        self.livox_rviz = Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['--display-config', rviz_config_path]
            )
        
        self.radar = Node(  package='charmie_radar',
                            executable='radar',
                            name='radar',
                            emulate_tty=True
                            )
        '''
        self.odometry_lidar = Node(package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/rf2o_laser_odometry/odom',
                    'publish_tf' : False,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 10.0}],
                )
        
        self.charmie_navigation = Node(package='charmie_navigation',
                    executable='navigation',
                    name='navigation',
                    emulate_tty=True
                    )
        
        ### JOY & GAMEPAD CONTROLLER
        # Compute config file path using LaunchConfiguration and TextSubstitution
        '''self.config_filepath = LaunchConfiguration('config_filepath', default=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('charmie_gamepad'), 'config', 'ps4')),
            TextSubstitution(text='.config.yaml')
        ])
        # joy_node
        self.joy = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        )

        # custom charmie_gamepad_node
        self.gamepad = Node(
            package='charmie_gamepad',
            executable='charmie_gamepad_node',
            name='charmie_gamepad_node',
            parameters=[self.config_filepath],
        )
        
        self.navigation_with_gamepad = Node(package='charmie_demonstration',
                executable='navigation_demonstration',
                name='navigation_demonstration',
                emulate_tty=True
                )
        
        self.task_with_gamepad = Node(package='charmie_demonstration',
                executable='task_demonstration',
                name='task_demonstration',
                emulate_tty=True
                )
        '''
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

        ### ADDITIONAL NON VISIBLE MAPS SERVER LAYER
        manual_obstacles_map_path = os.path.join(
            get_package_share_path('configuration_files'),
            'maps',
            'manual_obstacles.yaml'
        )

        self.manual_obstacles_map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='manual_obstacle_map_server',
            output='screen',
            parameters=[{
                'yaml_filename': manual_obstacles_map_path,
                'topic_name': '/manual_obstacles_map',
                'use_sim_time': False
            }]
        )
                
        ### LOCALIZATION

        # Exmaples of how the map should be added to launch file
        map_path = os.path.join(get_package_share_path('configuration_files'), 'maps', 'map.yaml')
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

        self.nav2_navigation_safety_inspection = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_path('charmie_description'), 'launch', 'navigation_inspection_launch.py')]),
                launch_arguments=[
                    ('use_sim_time', 'false')
                    ]
        )

        ### robot_localization node ###
        robot_localization_ekf_config_path = os.path.join(get_package_share_path('charmie_description'), 
                                'config', 'ekf.yaml')
        

        self.robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_ekf_config_path],
            )

        ### ROSBAG
        # Node to start rosbag recording
        self.rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],  # Command to start recording all topics
        name='rosbag_record_node',
        output='screen'
        )