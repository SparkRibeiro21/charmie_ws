import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import Command

from launch_ros.actions import Node
#from launch_ros.descriptions import ParameterValue
import launch_ros.descriptions

from launch.event_handlers import OnProcessStart

import xacro

def generate_launch_description():

    package_name='charmie_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name), "config", "controller_config.yaml")
    rviz2_file = os.path.join(get_package_share_directory(package_name), "config", "view_bot.rviz")
    #robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # robot_description = xacro.process_file(xacro_file).toxml() #SAPO


    delayed_actions = []

    # Add a half-second delay before launching each node
    delay = 0.5

    """ # SAPO
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{
            'robot_description': ParameterValue(
                value=Command(['ros2 param get --hide-type /robot_state_publisher robot_description']),
                value_type=str  # Set the value type explicitly to string
            )
        },
        controller_params_file]
    ) """


    """controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )"""

    """delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
        
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    
    delayed_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_trajectory_controller_spawner],
        )
    )"""

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    joint_state = Node(package='joint_state_publisher',
                       executable='joint_state_publisher',
                       name='joint_state_publisher',
                       output='screen',
                       parameters=[{'use_sim_time': False}])

    rviz2_spawner = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', 'src/charmie_bot/config/tiago_rviz.rviz'],
    )

    #create node debug_main
    debug_main = Node(package='charmie_debug',
                      executable='debug_main',
                      name = 'debug_main',
                      )
    
    #create node neck 
    neck = Node(package='charmie_neck_dynamixel',
                      executable='neck_dynamixel',
                      name = 'neck_dynamixel',
                      )

    #create node low_level 
    low_level = Node(package='charmie_low_level',
                      executable='low_level',
                      name = 'low_level',
                      )
    
    #create node speakers 
    speakers = Node(package='charmie_speakers',
                      executable='speakers',
                      name = 'speakers',
                      )
                      
    #create node speakers offline
    speakers_offline = Node(package='charmie_speakers_offline',
                      executable='speakers_offline',
                      name = 'speakers_offline',
                      )

    #create node audio 
    audio = Node(package='charmie_audio',
                      executable='audio',
                      name = 'audio',
                      )
    
    #create node lidar 
    lidar = Node(package='charmie_lidar_hokuyo',
                      executable='lidar_hokuyo',
                      name = 'lidar_hojuyo',
                      )
    
    #create node face 
    face = Node(package='charmie_face_shiningrgb',
                      executable='face_shiningrgb',
                      name = 'face_shining_rgb',
                      )
    
    #create node camera 
    camera = Node(package='realsense2_camera',
                      executable='realsense2_camera_node',
                      name = 'realsense2_camera_node',
                      arguments=['--ros-args', '-p', 'align_depth.enable:=true', '-p', 'pointcloud.enable:=true']
                      )
    
    #create node PS4
    ps4 = Node(package='charmie_ps4_controller',
                      executable='ps4_controller',
                      name = 'ps4_controller',
                      )
    
    #create node odometry
    odometry = Node(package='charmie_odometry',
                      executable='odometry',
                      name = 'odometry',
                      )
    
    #create node obstacles
    obstacles = Node(package='charmie_obstacles',
                      executable='obstacles',
                      name = 'obstacles',
                      )
    #create navigation obstacle
    navigation = Node(package='charmie_navigation_sdnl',
                      executable='navigation',
                      name='navigation',
                      )
                      
    #charge map 
    map_server = Node(package='nav2_map_server',
                      executable='map_server',
                      name='map_server',
                      arguments=['--ros-args', '-p', 'yaml_filename:=./src/charmie_bot/maps_rviz/house_lar_29.yaml', '-p', 'use_sim_time:=false']
                      )
                      
    #run map
    calling_map = Node(package='nav2_util',
    			executable='lifecycle_bringup',
                       name='map_server_node',
                       )
                    
    #charge amcl
    amcl = Node(package='nav2_amcl',
                      executable='amcl',
                      name='amcl',
                      arguments=['--ros-args', '-p', 'yaml_filename:=./src/charmie_bot/config/amcl_config.yaml', '-p', 'use_sim_time:=false']
                      )
           
    #run map
    calling_amcl = Node(package='nav2_util',
    			executable='lifecycle_bringup',
                       name='amcl',
                       )
    

    # Create delayed actions for the first four nodes
    for node in [audio, low_level, odometry, lidar, obstacles, navigation]: #---------> CHANGE ME
        delayed_actions.append(TimerAction(period=delay, actions=[node]))
        delay += 0.5
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        lidar,
        #delayed_controller_manager,
        #delayed_joint_state_broadcaster_spawner,
        #delayed_joint_trajectory_controller_spawner,

        #*delayed_actions,
        
        #map_server,
        #amcl,
        #speakers,
        #speakers_offline,
        #face,
        #neck,
        #camera,
        #debug_main,
        #ps4,
        #calling_map,
        #calling_amcl,
        rviz2_spawner
    ])
