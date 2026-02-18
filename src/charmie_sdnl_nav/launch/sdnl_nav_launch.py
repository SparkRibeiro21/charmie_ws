from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    sdnl_params_yaml_path = os.path.join(
        get_package_share_path('charmie_description'),
        'config',
        'sdnl_nav_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='charmie_sdnl_nav',
            executable='sdnl_nav',
            name='sdnl_nav',
            output='screen',
            parameters=[sdnl_params_yaml_path]
        )
    ])