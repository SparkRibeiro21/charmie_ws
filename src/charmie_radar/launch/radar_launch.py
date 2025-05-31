from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='charmie_radar',
            executable='radar',
            name='radar_node',
            output='screen',
            parameters=[{
                'radar_config': os.path.join(get_package_share_path('charmie_description'), 'config', 'radar_params.yaml')
            }]
        )
    ])