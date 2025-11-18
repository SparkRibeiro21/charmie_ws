from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    use_real_hardware_arg = DeclareLaunchArgument(
        'use_real_hardware',
        default_value='false',
        description='Use real hardware instead of mock components'
    )
    
    # Get launch configurations
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    
    # Get package paths
    charmie_description_share = FindPackageShare('charmie_description')
    charmie_moveit_config_share = FindPackageShare('charmie_moveit_config')

    # Build MoveIt config
    moveit_config = MoveItConfigsBuilder("charmie", package_name="charmie_moveit_config").to_moveit_configs()
    
    
    # Define file paths
    urdf_path = PathJoinSubstitution([
        charmie_description_share,
        'urdf',
        'charmie_real.urdf.xacro'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        charmie_description_share,
        'rviz',
        'moveit.rviz'
    ])
    
    ros2_controllers_path = PathJoinSubstitution([
        charmie_moveit_config_share,
        'config',
        'ros2_controllers.yaml'
    ])
    
    # Generate robot description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        urdf_path, ' ',
        'use_real_hardware:=', use_real_hardware
    ])
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # ROS2 Control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_path],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # xArm6 Controller
    xarm6_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm6_controller'],
        output='screen'
    )
    
    # xArm Gripper Controller
    xarm_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm_gripper_controller'],
        output='screen'
    )
    
    # MoveIt
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                charmie_moveit_config_share,
                'launch',
                'move_group.launch.py'
            ])
        ])
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.sensors_3d,
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_real_hardware_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        xarm6_controller_spawner,
        xarm_gripper_controller_spawner,
        move_group_launch,
        rviz_node
    ])