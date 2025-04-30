### This is an example of how to create a launch file in ROS2 using Python. The goal of a launch file is to initialize multiple modules by just running one line in the cmd line.
### First of all, in order to call a launch file from a package in ROS2 written in python, we need to change the "setup.py" file inside the package. So it is needed to add
### at the end of data_files, the following line (inside ""): "(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),". Also, in the same file, it is needed to add the import
### at the beggining: "import os"
### and from glob import glob"

### Then we can start writing in the launch file. During this script there are some commentaries that may help to better understand some things. Anyway, it is advisable to read
### the official documentation provided by ROS2: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html


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

    ### ------------------- DEFINING PATHS ------------------- ###
    package_name='charmie_debug' #<--- CHANGE ME FOR THE CORRECT PACKAGE NAME I AM LOCATED AT
    pkg_path = os.path.join(get_package_share_directory(package_name))
    bag_path = pkg_path + '/../../../../src/bag_files/' + package_name + '_' + str(now) #<--- CHANGE ME FOR THE DIRECTORY WHERE YOU WANT TO STORE THE ROSBAGS

    #This following lines are to able the use of RVIZ and to have a visualization of the robot in RVIZ

    pkg_path_bot = os.path.join(get_package_share_directory('charmie_bot'))
    xacro_file = os.path.join(pkg_path_bot,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    ### ---------------------------- // ----------------------------- ###



    # Add a half-second delay before launching each node
    delay = 0.5
    delayed_actions = []



    ### ------------------- CREATE NODES ------------------- ###
    ### The following lines are to create the variables corresponding to the nodes we want to launch.
    ### Some of them start with "Node" and others with "ExecuteProcess". The difference is that 
    ### the ones starting with Node are running nodes from the workspace, while the ExecuteProcess
    ### ones are running commands directly from the command line.


    # Create a robot_state_publisher node
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

    # start recording bag
    # The exclude is to avoid crashes while using intel rs cameras. 
    # If you're not using them in your launch file, you can just delete thatr part
    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', bag_path, '-a', '--exclude', "(/CHARMIE/D455_head/aligned_depth_to_color/image_raw/theora|/CHARMIE/D455_head/aligned_depth_to_color/image_raw/compressedDepth|/CHARMIE/D455_head/aligned_depth_to_color/image_raw/compressed|/CHARMIE/D455_head/color/image_raw/compressed|/CHARMIE/D455_head/color/image_raw/compressedDepth|/CHARMIE/D455_head/color/image_raw/theora|/CHARMIE/D455_head/depth/image_rect_raw/compressed|/CHARMIE/D455_head/depth/image_rect_raw/compressedDepth|/CHARMIE/D455_head/depth/image_rect_raw/theora)"], #<----- change me if you want to record just some specific topics (-a)
                            output= 'screen')
    
    # ros2 bag record -a --exclude "(/CHARMIE/D455_head/aligned_depth_to_color/image_raw/theora|/CHARMIE/D455_head/aligned_depth_to_color/image_raw/compressedDepth|/CHARMIE/D455_head/aligned_depth_to_color/image_raw/compressed|/CHARMIE/D455_head/color/image_raw/compressed|/CHARMIE/D455_head/color/image_raw/compressedDepth|/CHARMIE/D455_head/color/image_raw/theora|/CHARMIE/D455_head/depth/image_rect_raw/compressed|/CHARMIE/D455_head/depth/image_rect_raw/compressedDepth|/CHARMIE/D455_head/depth/image_rect_raw/theora)"

    
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
    ### ---------------------------- // ----------------------------- ###






    ### ------------------- CHOOSE ORDER TO LAUNCH NODES ------------------- ###
    ### This cycle is for cases where the order of launching the nodes is important due to dependancies of some nodes on others.
    ### Like this, each node on the list "delayed_actions" will be launched with a time difference of "delay".
    ### In this project, there are some nodes that depend on others, and those are explained at the end of this script.
    for node in [node_robot_state_publisher, joint_state, low_level, localization, navigation, lidar, obstacles, audio, speakers_offline, neck]: #---------> CHANGE ME FOR THE NODES YOU WANT TO LAUNCH DELAYED
        delayed_actions.append(TimerAction(period=delay, actions=[node]))
        delay += 0.5

    return LaunchDescription([
        ### In here the Nodes or Processes created before are called to run, and so the nodes can be started from the launch file.
        #*delayed_actions,
        
        #map_server_cmd,
        #map_server_with_params_cmd,
        #amcl_cmd,
        #amcl_with_params_cmd,
        #speakers,
        #audio,
        rosbag,
        #debug_main
        #face,
        #camera,
        #yolopose
        #rviz2
    ])



    ### ------------------- NODE DEPENDENCIES ------------------- ###
    ### odometry depends on low_level 
    ### obstacles depend on lidar
    ### audio should be launched before the neck because of possible vibrations caused by launching the neck that might influence the audio on microphone