from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files
    
    return LaunchDescription([
        std_lf.robot_state_publisher_real_node,
        #std_lf.joint_state_publisher,
        std_lf.joint_state_publisher_gui,
        std_lf.static_transforms_launch,
        std_lf.rviz2_basic_node,
        std_lf.xarm_ros2_control,
        std_lf.xarm_controller_spawner
    ])