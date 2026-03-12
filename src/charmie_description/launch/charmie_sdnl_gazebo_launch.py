from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files
        
    return LaunchDescription([
        std_lf.robot_state_publisher_gazebo_node,
        std_lf.rviz2_basic_node,
        std_lf.twist_mux_node,
        std_lf.spawn_entity,
        std_lf.diff_drive_spawner,
        std_lf.joint_broad_spawner,
        std_lf.gazebo,
        std_lf.radar,
        std_lf.marker_arrays_debug,
        # std_lf.odometry_lidar,
    ])