from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files
    
    return LaunchDescription([
        std_lf.gui,
        # std_lf.audio,
        # LaunchDescription(std_lf.declared_arm_arguments + [std_lf.robot_arm_driver_launch]),
        # std_lf.arm,
        std_lf.speakers,
        # std_lf.charmie_multi_camera_launch_description,
        std_lf.low_level,
        # std_lf.face,
        # std_lf.neck,
        std_lf.joy,
        std_lf.gamepad,
        # std_lf.yolo_objects,
        # std_lf.yolo_pose,
        # std_lf.lidar,
        # std_lf.llm
        # std_lf.obstacles,
        # std_lf.navigation
        
        # std_lf.node_robot_state_publisher,
        # std_lf.joint_state,
    ])
