from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

   
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


    home = str(Path.home())

    # robot driver launch
    # xarm_api/launch/_robot_driver.launch.py
    robot_driver_launch = IncludeLaunchDescription(
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

    ps4_controller = Node(package='charmie_ps4_controller',
                executable='ps4_controller',
                name='ps4_controller',
                parameters=[
                    {'control_arm': True},
                    {'control_face': True},
                    {'control_motors': True},
                    {'control_neck': True},
                    {'control_rgb': False},
                    {'control_set_movement': True},
                    {'control_speakers': True},
                    {'control_torso': True},
                    {'control_wait_for_end_of_navigation': True},
                    ],
                emulate_tty=True
                )
    
    speakers = Node(package='charmie_speakers',
                        executable='speakers',
                        name='speakers',
                        emulate_tty=True
                        )
    
    neck = Node(package='charmie_neck_dynamixel',
                executable='neck_dynamixel',
                name='neck_dynamixel',
                emulate_tty=True
                )
    
    low_level = Node(package='charmie_low_level',
                executable='low_level',
                name='low_level',
                emulate_tty=True
                )
    
    face = Node(package='charmie_face',
                executable='face',
                name='face',
                parameters=[
                    {'show_speech': True},
                    {'after_speech_timer': 1.0},
                    {'initial_face': 'charmie_face'},
                    ],
                emulate_tty=True
                )

    arm = Node(package='charmie_arm_ufactory',
                        executable='arm_hello',
                        name='arm_hello',
                emulate_tty=True
                        )
    

    return LaunchDescription([
        # LaunchDescription(declared_arguments + [robot_driver_launch]),
        # face,
        # speakers,
        neck,
        low_level,
        ps4_controller,
        # arm,
    ])
