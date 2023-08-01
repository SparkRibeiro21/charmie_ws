import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
from datetime import datetime

def generate_launch_description():
    now = datetime.now()
    package_name='charmie_receptionist' #<--- CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(package_name))
    bag_path = pkg_path + '/../../../../src/bag_files/' + package_name + '_' + str(now) #<--- CHANGE ME

    # Process the URDF file
    pkg_path_bot = os.path.join(get_package_share_directory('charmie_bot'))
    

    xacro_file = os.path.join(pkg_path_bot,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    delayed_actions = []
    delayed_diagnostic = []

    # Add a half-second delay before launching each node
    delay = 0.5
    delay_diagnostic = 2.0

 
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
    
    #publishes the RVIZ2 parameters
    rviz2 = Node(package='rviz2',
                 executable='rviz2',
                 name='rviz2',
                 arguments=['-d', [os.path.join(pkg_path, 'config', 'view_bot.rviz')]]
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
                      executable='speakers',
                      name = 'speakers',
                      )

    #create node audio 
    audio = Node(package='charmie_audio',
                      executable='audio',
                      name = 'audio',
                      )
    
    #create node lidar 
    lidar = Node(package='charmie_lidar_hokuyo',
                      executable='lidar_hokuyo',
                      name = 'lidar_hokuyo',
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
    
    #create navigation node
    navigation = Node(package='charmie_navigation_sdnl',
                      executable='navigation',
                      name='navigation',
                      )
                      
    #create door_start node
    door_start = Node(package='charmie_door_start',
                      executable='door_start',
                      name='door_start',
                      )
    #create node YOLO_Pose
    yolopose = Node(package='yolopose',
                    executable='yolo_posev8',
                    name='yolo_posev8',
                    )
    
    #create node localization
    localization = Node(package='charmie_localisation',
                        executable='localisation',
                        name='localisation',
                        )

    #create node inspection
    inspection = Node(package='charmie_inspection',
                     executable='inspection',
                     name='inspection',
                     )
    #create node inspection
    diagnostics = Node(package='charmie_diagnostics',
                     executable='diagnostics',
                     name='diagnostics',
                     )
    
    receptionist = Node(package='charmie_receptionist',
                     executable='receptionist',
                     name='receptionist',
                     )
    
    #start recording bag
    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_path, '-a'], #<----- change me
                            output= 'screen')
    
    # Command to launch the map_server node
    map_server_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        output='screen'
    )

    # Command to launch the map_server with specified parameters
    map_server_with_params_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', 'yaml_filename:=./robocup_house_1_save.yaml', '-p', 'use_sim_time:=false'],
        output='screen'
    )

    # Command to launch the amcl node
    amcl_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
        output='screen'
    )

    # Command to launch the amcl node with specified parameters
    amcl_with_params_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args', '-p', 'use_sim_time:=false'],
        output='screen'
    )

    """ # Create the map_server node with parameters
    map_server_with_params = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_with_params',
        output='screen',
        arguments=['--ros-args', '-p', 'yaml_filename:=./robocup_house_1_save.yaml', '-p', 'use_sim_time:=false'],
    )

    # Create the amcl node with parameters
    amcl_with_params = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl_with_params',
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=false'],
    )

    map_server = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='map_server',
        output='screen',
        arguments=['map_server'],
    )

    # Create the amcl node
    amcl = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='amcl',
        output='screen',
        arguments=['amcl'],
    )
 """
    
    # Create delayed actions for the first four nodes
    for node in [diagnostics,
                 map_server_cmd, map_server_with_params_cmd, amcl_cmd, amcl_with_params_cmd, 
                 low_level, odometry, localization, navigation, lidar, obstacles, audio, speakers_offline, neck]: #---------> CHANGE ME
        delayed_actions.append(TimerAction(period=delay, actions=[node]))
        delay += 0.5
    

    for node in [diagnostics]: #---------> CHANGE ME
        delayed_diagnostic.append(TimerAction(period=delay, actions=[node]))
        delay_diagnostic += 2.0

    return LaunchDescription([
        
        node_robot_state_publisher,
        joint_state,

        *delayed_actions,
        
        
        
        #low_level,
        #rosbag,
        #debug_main,
        #face,
        #camera,
        #yolopose,
        #*delayed_diagnostic,
        receptionist,
        
        rviz2
        
    ])
