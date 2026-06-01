from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Static transform to use just odom in localization (mainly for unmapped environments)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_just_odom',
            arguments=[
                '0',    # x
                '0',    # y
                '0.05', # z # by default in nav2, the odom TF should be at the same height as base_link. Therefore we need to add 0.05 which is the same value present in the URDF.
                '0',    # roll
                '0',    # pitch
                '0',    # yaw
                'map',  # parent frame
                'odom'  # child frame
            ]
        )

    ])
