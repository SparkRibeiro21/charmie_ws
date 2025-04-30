from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    charmie_bot_real_launch_file = PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('charmie_bot'), 'launch', 'launch_real.launch.py'
    )])

    slam_launch_file = PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
    )])

    home = str(Path.home())
    midpath = "charmie_ws/src/charmie_bot/maps_rviz"
    complete_path = home+'/'+midpath+'/'+'FNR24_v0.yaml'

    mapper_params_file = LaunchConfiguration('params_file', default='src/charmie_bot/config/mapper_params_online_async.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    lifecycle_nodes = ['map_server', 'amcl']


    # Use IncludeLaunchDescription to include the launch file
    charmie_bot_real_launch_description = IncludeLaunchDescription(charmie_bot_real_launch_file)

    load_map = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                # {'yaml_filename': './src/charmie_bot/maps_rviz/LAR_pre_nacional_2024_v2.yaml'},
                {'yaml_filename': complete_path},
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
    
    """ amcl_map_server_launch = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'node_names': lifecycle_nodes}]
    ) """

    amcl_map_server_launch = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='lifecycle_bringup',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'node_names': lifecycle_nodes}]
    )

    bringup_map_server = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        output='screen'
    )

    bringup_amcl = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
        output='screen'
    )
    
    # Add a half-second delay before launching each node
    delay = 0.5
    delay_2 = 2.0
    delayed_actions = []
    delay_amcl_map_server = []

    for node in [low_level, ps4]: #---------> CHANGE ME
        delayed_actions.append(TimerAction(period=delay, actions=[node]))
        delay += 0.5

    return LaunchDescription([
        *delayed_actions, 
        charmie_bot_real_launch_description,
        load_map,
        load_amcl,
        # bringup_map_server,
        # bringup_amcl
        # amcl_map_server_launch,
    ])