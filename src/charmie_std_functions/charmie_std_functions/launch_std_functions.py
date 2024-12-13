from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node
from pathlib import Path

import xacro
from datetime import datetime
import os

class LaunchStdFunctions():

    def __init__(self):
        
        # Process the URDF file
        pkg_path_bot = os.path.join(get_package_share_directory('charmie_bot'))
        
        xacro_file = os.path.join(pkg_path_bot,'description','robot.urdf.xacro')
        robot_description_config = xacro.process_file(xacro_file)
        
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
 
        # Create a robot_state_publisher node
        params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
        
        self.node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        )

        #Publishes the joint_states of the robot
        self.joint_state = Node(package='joint_state_publisher',
                       executable='joint_state_publisher',
                       name='joint_state_publisher',
                       output='screen',
                       parameters=[{'use_sim_time': True}])

        charmie_multi_camera_launch_file = PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'charmie_multi_camera_launch.py'
        )])

        # Use IncludeLaunchDescription to include the launch file
        self.charmie_multi_camera_launch_description = IncludeLaunchDescription(charmie_multi_camera_launch_file)

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
        
        self.navigation = Node(package='charmie_navigation_sdnl',
                    executable='navigation',
                    name='navigation',
                    emulate_tty=True
                    )
        
        self.ps4_controller = Node(package='charmie_ps4_controller',
                    executable='ps4_controller',
                    name='ps4_controller',
                    )
        
        self.llm = Node(package='charmie_llm',
                    executable='llm',
                    name='llm',
                    )