from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files
    
    return LaunchDescription([
        std_lf.gui,
        std_lf.rviz2_basic_node,
        # std_lf.audio,
        LaunchDescription(std_lf.declared_arm_arguments + [std_lf.robot_arm_driver_launch]),
        std_lf.arm,
        std_lf.speakers,
        std_lf.charmie_multi_camera_launch_description,
       #std_lf.charmie_orbbec_base_camera_launch_description,
        std_lf.low_level,
        # std_lf.face,
        std_lf.neck,

        # std_lf.point_cloud,
        std_lf.yolo_objects,
        # std_lf.yolo_pose,
        #std_lf.lidar,
        # std_lf.llm
        
        ### std_lf.obstacles,
        ### std_lf.navigation
        
        std_lf.robot_state_publisher_real_node,
        std_lf.static_transforms_launch
    ])