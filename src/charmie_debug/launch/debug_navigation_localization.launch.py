from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files
    
    return LaunchDescription([

        #####################################################################
        # Robot State and Static Transforms                                 #
        #####################################################################
        std_lf.robot_state_publisher_real_node,
        std_lf.static_transforms_launch,
        
        #####################################################################
        # Visualization and Debugging                                       # 
        #####################################################################
        std_lf.rviz2_nav2_node,
        std_lf.gui,
        std_lf.marker_arrays_debug,
        # std_lf.task_with_gamepad, # debug mode where task is selected via gamepad

        ######################################################################
        # Actuators                                                          #
        ######################################################################
        # LaunchDescription(std_lf.declared_arm_arguments + [std_lf.robot_arm_driver_launch]),
        # std_lf.arm,
        std_lf.speakers,
        # std_lf.neck,
        # std_lf.face,
        std_lf.low_level,
        
        #####################################################################
        # Sensors                                                           # 
        #####################################################################
        std_lf.lidar,
        std_lf.lidar_bottom,
        # std_lf.livox_driver,
        std_lf.radar,
        # std_lf.audio,
        # std_lf.sound_classification,
        
        #####################################################################
        # Localization                                                      # 
        #####################################################################
        std_lf.odometry_lidar,
        std_lf.robot_localization,
        std_lf.charmie_localisation,

        #####################################################################
        # Navigation                                                        # 
        #####################################################################
        std_lf.manual_obstacles_map_server,
        std_lf.delayed_nav2_localization,        
        std_lf.nav2_navigation,
        std_lf.charmie_navigation,

        #####################################################################
        # Cameras                                                           # 
        #####################################################################
        # CAMS have to be after nav2 otherwise map does not show up in rviz
        # std_lf.charmie_multi_camera_launch_description,
        # std_lf.charmie_orbbec_base_camera_launch_description,

        #####################################################################
        # Teleoperation                                                     # 
        #####################################################################
        # std_lf.joy,
        # std_lf.gamepad,
        # std_lf.navigation_with_gamepad,

        #####################################################################
        # Detections                                                        # 
        #####################################################################
        # std_lf.yolo_objects,
        # std_lf.yolo_pose,
        # std_lf.tracking,
        # std_lf.llm,

    ])