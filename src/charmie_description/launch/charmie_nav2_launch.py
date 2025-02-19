from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files

    return LaunchDescription([
        std_lf.robot_state_publisher_real_node,
        std_lf.static_transforms_launch,
        std_lf.rviz2_nav2_node,
        
        std_lf.odometry_lidar,
        # std_lf.gui,
        # std_lf.speakers,
        std_lf.lidar,
        std_lf.lidar_bottom,
        std_lf.low_level,
        # std_lf.ps4_controller,
        
    ])