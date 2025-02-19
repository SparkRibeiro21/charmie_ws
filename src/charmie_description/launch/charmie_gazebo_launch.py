from launch import LaunchDescription
from charmie_std_functions.launch_std_functions import LaunchStdFunctions

def generate_launch_description():

    std_lf = LaunchStdFunctions() # From charmie_std_functions - Standardizes launch files

    return LaunchDescription([

        std_lf.robot_state_publisher_gazebo_node,
        std_lf.joint_state_publisher_gui,
        std_lf.rviz2_basic_node,
        std_lf.spawn_entity,
        std_lf.gazebo,
        std_lf.odometry_lidar
    ])