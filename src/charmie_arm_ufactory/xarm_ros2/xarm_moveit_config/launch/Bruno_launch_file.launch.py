from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xarm_moveit_config_pkg_dir = get_package_share_directory('xarm_moveit_config')

    # Define arguments for the launch file
    params_file = LaunchConfiguration('params_file', default=xarm_moveit_config_pkg_dir + '/config/xarm6/sensor_intel_hand_pointcloud.yaml')
    octomap_frame_param = DeclareLaunchArgument('octomap_frame', default_value='odom_combined')
    octomap_resolution_param = DeclareLaunchArgument('octomap_resolution', default_value='0.05')
    max_range_param = DeclareLaunchArgument('max_range', default_value='5.0')

    # Load parameters from the YAML file
    load_params_node = Node(
        package='xarm_moveit_config',
        executable='xarm_moveit_config',
        namespace='',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        octomap_frame_param,
        octomap_resolution_param,
        max_range_param,
        load_params_node
    ])