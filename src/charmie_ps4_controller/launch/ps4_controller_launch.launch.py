from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

   
    ps4_controller = Node(package='charmie_ps4_controller',
                executable='ps4_controller',
                name='ps4_controller',
                )
    
    speakers = Node(package='charmie_speakers',
                        executable='speakers',
                        name='speakers',
                        )
    
    neck = Node(package='charmie_neck_dynamixel',
                executable='neck_dynamixel',
                name='neck_dynamixel',
                )
    
    low_level = Node(package='charmie_low_level',
                executable='low_level',
                name='low_level',
                )
    
    face = Node(package='charmie_face',
                executable='face',
                name='face',
                )
    

    return LaunchDescription([
        speakers,
        face,
        # neck,
        # low_level,
        ps4_controller,
    ])
