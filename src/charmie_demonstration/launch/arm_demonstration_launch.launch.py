import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node

import xacro
from datetime import datetime

def generate_launch_description():
    now = datetime.now()
    package_name='charmie_demonstration' #<--- CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(package_name))

    # Process the URDF file
    pkg_path_bot = os.path.join(get_package_share_directory('charmie_bot'))
    
    xacro_file = os.path.join(pkg_path_bot,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.219',
            description='IP address by which the robot can be reached.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'report_type',
            default_value='normal',
            description='Tcp report type, default is normal, normal/rich/dev optional.',
        )
    )
    declared_arguments.append(
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

    # robot driver launch
    # xarm_api/launch/_robot_driver.launch.py
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/charmie/charmie_ws/src/charmie_arm_ufactory/xarm_ros2/xarm_api/launch/_robot_driver.launch.py'),
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
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #Publishes the joint_states of the robot
    joint_state = Node(package='joint_state_publisher',
                       executable='joint_state_publisher',
                       name='joint_state_publisher',
                       output='screen',
                       parameters=[{'use_sim_time': True}])
    
    pick_and_place_demonstration = Node(package='charmie_demonstration',
                     executable='pick_place_demonstration',
                     name='pick_place_demonstration',
                     )
    
    speakers = Node(package='charmie_speakers',
                        executable='speakers',
                        name='speakers',
                        )

    arm = Node(package='charmie_arm_ufactory',
                        executable='arm_ufactory',
                        name='arm_ufactory',
                        )
    
    neck = Node(package='charmie_neck_dynamixel',
                executable='neck_dynamixel',
                name='neck_dynamixel',
                )
    
    low_level = Node(package='charmie_low_level',
                executable='low_level',
                name='low_level',
                )
    
    ps4_controller = Node(package='charmie_ps4_controller',
                executable='ps4_controller',
                name='ps4_controller',
                )

    return LaunchDescription([
        speakers,
        LaunchDescription(declared_arguments + [robot_driver_launch]),
        node_robot_state_publisher,
        joint_state,
        neck,
        low_level,
        arm,
        ps4_controller,
        pick_and_place_demonstration,
    ])
