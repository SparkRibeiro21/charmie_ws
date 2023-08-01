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
    package_name='charmie_carry_luggage' #<--- CHANGE ME
    pkg_path = os.path.join(get_package_share_directory(package_name))
    bag_path = pkg_path + '/../../../../src/bag_files/' + package_name + '_' + str(now) #<--- CHANGE ME

    # Process the URDF file
    pkg_path_bot = os.path.join(get_package_share_directory('charmie_bot'))
    

    xacro_file = os.path.join(pkg_path_bot,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    delayed_actions = []

    # Add a half-second delay before launching each node
    delay = 0.5

 
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
    carry_luggage = Node(package='charmie_carry_luggage',
                     executable='charmie_carry_luggage',
                     name='carry_my_luggage',
                     )
    
    #start recording bag
    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_path, '-a'], #<----- change me
                            output= 'screen')
    
    # Create delayed actions for the first four nodes
    for node in [low_level, odometry, localization, navigation, camera, lidar, obstacles]: #---------> CHANGE ME
        delayed_actions.append(TimerAction(period=delay, actions=[node]))
        delay += 0.5
    

    return LaunchDescription([
        node_robot_state_publisher,
        joint_state,
        
        *delayed_actions,
        
        #rosbag,
        #debug_main,
        #face,
        #yolopose,
        #rviz2,
        neck
        
    ])
