from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from charmie_std_functions.launch_std_functions import LaunchStdFunctions


def generate_launch_description():

    std_lf = LaunchStdFunctions()

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
    moveit_config = (MoveItConfigsBuilder("charmie", package_name="charmie_moveit_config")
                     .planning_pipelines("ompl")
                     .to_moveit_configs())
    
    
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
        parameters=[
            {'robot_description': robot_description},
            ros2_controllers_path],
        # remappings=[('~/robot_description', '/robot_description')],
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
        ]),
        launch_arguments={
            'publish_monitored_planning_scene': 'false',
        }.items()
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        output='screen'
    )

    commander_node = Node(
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

    neck_node = Node(
        package='charmie_neck_dynamixel',
        executable='neck_dynamixel',
        name='neck_dynamixel',
        emulate_tty=True,
        condition=IfCondition(use_real_hardware),
    )
    
    low_level_node = Node(
        package='charmie_low_level',
        executable='low_level_stream',
        name='low_level_stream',
        emulate_tty=True,
        condition=IfCondition(use_real_hardware),
    )

    map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_base_camera',
            arguments=[
                '0',  # x
                '0',  # y
                '0',  # z
                '0',  # roll
                '0',  # pitch
                '0',  # yaw
                'map',  # parent frame
                'odom'  # child frame
            ]
        )
    
    return LaunchDescription([

        #####################################################################
        # Robot State and Static Transforms                                 #
        #####################################################################
        std_lf.robot_state_publisher_real_node,
        std_lf.static_transforms_launch,
        use_real_hardware_arg,
        std_lf.static_transforms_just_odom_launch, #Temporary

        #####################################################################
        # Visualization and Debugging                                       # 
        #####################################################################
        rviz_node,
        # std_lf.rviz2_nav2_node,
        std_lf.gui,
        std_lf.marker_arrays_debug,

        ######################################################################
        # Actuators                                                          #
        ######################################################################
        LaunchDescription(std_lf.declared_arm_arguments + [std_lf.robot_arm_driver_launch]),
        neck_node,
        low_level_node,

        #####################################################################
        # Localization                                                      # 
        #####################################################################
        std_lf.odometry_lidar,
        std_lf.robot_localization,
        std_lf.charmie_localisation,

        #####################################################################
        # Navigation                                                        # 
        #####################################################################
        std_lf.manual_obstacles_map_server,
        std_lf.delayed_nav2_localization,        
        std_lf.nav2_navigation,
        std_lf.charmie_navigation,

        #####################################################################
        # Cameras                                                           # 
        #####################################################################
        # std_lf.charmie_multi_camera_launch_description,

        #####################################################################
        # Detections                                                        # 
        #####################################################################
        std_lf.yolo_objects,

        #####################################################################
        # MoveIt                                                            # 
        #####################################################################
        ros2_control_node,
        joint_state_broadcaster_spawner,
        xarm6_controller_spawner,
        # xarm_gripper_controller_spawner,
        move_group_launch,
        commander_node,
    ])