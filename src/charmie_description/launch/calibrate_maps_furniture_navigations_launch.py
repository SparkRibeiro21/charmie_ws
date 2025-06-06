from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files

    return LaunchDescription([
        # std_lf.robot_state_publisher_real_node,
        # std_lf.static_transforms_launch,
        std_lf.rviz2_calib_map_furniture_navigations_node,
        std_lf.marker_arrays_debug,
        
        # std_lf.odometry_lidar,
        # std_lf.gui,
        # std_lf.speakers,
        # std_lf.lidar,
        # std_lf.lidar_bottom,
        # std_lf.low_level,
        # std_lf.charmie_multi_camera_launch_description,
        # std_lf.neck,

        # std_lf.robot_localization,
        # std_lf.charmie_localisation,

        std_lf.delayed_nav2_localization,        
        # std_lf.nav2_navigation,

        # std_lf.charmie_orbbec_base_camera_launch_description,
        
    ])