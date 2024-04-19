from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    charmie_bot_real_launch_file = PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('charmie_bot'), 'launch', 'launch_real.launch.py'
    )])

    slam_launch_file = PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
    )])

    mapper_params_file = LaunchConfiguration('params_file', default='src/charmie_bot/config/mapper_params_online_async.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Use IncludeLaunchDescription to include the launch file
    charmie_bot_real_launch_description = IncludeLaunchDescription(charmie_bot_real_launch_file)

    load_map = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'yaml_filename': './src/charmie_bot/maps_rviz/LAR_pre_nacional_2024_v2.yaml'},
                {'use_sim_time': False}
            ]
        )
    
    """ nav2_util_launch = Command(
        command=['ros2 run nav2_util lifecycle_bringup map_server']  # Your original command
    ) """
    
    load_amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    
    """ amcl_launch = Command(
        command=['ros2 run nav2_util lifecycle_bringup amcl']  # Your original command
    ) """

    #create node odometry
    odometry = Node(package='charmie_odometry',
                      executable='odometry',
                      name = 'odometry',
                      )

    #create node low_level 
    low_level = Node(package='charmie_low_level',
                      executable='low_level',
                      name = 'low_level',
                      )
    
    #create node PS4
    ps4 = Node(package='charmie_ps4_controller',
                      executable='ps4_controller',
                      name = 'ps4_controller',
                      )
    
    # Add a half-second delay before launching each node
    delay = 0.5
    delayed_actions = []

    for node in [low_level, odometry, ps4]: #---------> CHANGE ME
        delayed_actions.append(TimerAction(period=delay, actions=[node]))
        delay += 0.5

    return LaunchDescription([
        *delayed_actions, 
        charmie_bot_real_launch_description,
        load_map,
        load_amcl,
    ])