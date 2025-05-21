from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files
    
    return LaunchDescription([
        std_lf.robot_state_publisher_real_node,
        std_lf.static_transforms_launch,
        std_lf.rviz2_nav2_node,
        std_lf.gui,
        std_lf.marker_arrays_debug,

        LaunchDescription(std_lf.declared_arm_arguments + [std_lf.robot_arm_driver_launch]),
        std_lf.arm,
        
        std_lf.low_level,
        std_lf.lidar,
        std_lf.lidar_bottom,
        
        std_lf.speakers,
        std_lf.neck,
        ###std_lf.face,
        
        std_lf.odometry_lidar,
        std_lf.robot_localization,
        std_lf.charmie_localisation,

        std_lf.delayed_nav2_localization,        
        std_lf.nav2_navigation,

        # CAMS have to be after nav2 otherwise map does not show up in rviz
        std_lf.charmie_multi_camera_launch_description,
        std_lf.charmie_orbbec_base_camera_launch_description,

        std_lf.yolo_objects,

    ])