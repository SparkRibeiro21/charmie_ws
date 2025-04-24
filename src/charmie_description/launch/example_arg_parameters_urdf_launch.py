from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

### How can be called via CLI:
### ros2 launch charmie_description example_arg_parameters_urdf_launch.py wheel_radius:=0.2

def generate_launch_description():
    # Declare launch arguments
    wheel_radius_arg = DeclareLaunchArgument('wheel_radius', default_value='0.1')
    head_version_arg = DeclareLaunchArgument('head_version', default_value='2')
    
    # Use launch configuration variables
    wheel_radius = LaunchConfiguration('wheel_radius')
    head_version = LaunchConfiguration('head_version')

    # Path to the xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('charmie_description'),
        'urdf',
        'charmie_real.urdf.xacro'
    ])

    # Generate robot_description from xacro with parameters
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' wheel_radius:=', wheel_radius,
        ' head_version:=', head_version
    ])

    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        wheel_radius_arg,
        head_version_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        )
    ])


