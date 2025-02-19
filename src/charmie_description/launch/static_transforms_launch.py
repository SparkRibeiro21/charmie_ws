from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform for the left wheel
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_left_wheel',
            arguments=[
                '0',  # x
                '0.255',  # y
                '0.05',  # z
                '0',  # roll
                '0',  # pitch
                '0',  # yaw
                'base_link',  # parent frame
                'left_wheel_link'  # child frame
            ]
        ),

        # Static transform for the right wheel
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_right_wheel',
            arguments=[
                '0',  # x
                '-0.255',  # y
                '0.05',  # z
                '0',  # roll
                '0',  # pitch
                '0',  # yaw
                'base_link',  # parent frame
                'right_wheel_link'  # child frame
            ]
        ),

        # Static transform for the torso and legs
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_torso_legs',
            arguments=[
                '0',  # x
                '0',  # y
                '0.475',  # z
                '0',  # roll
                '0',  # pitch
                '0',  # yaw
                'base_link',  # parent frame
                'legs_link'  # child frame
            ]
        ),

        # Static transform for torso to torso link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_torso_torso',
            arguments=[
                '0',  # x
                '0',  # y
                '0.25',  # z
                '0',  # roll
                '0.15708',  # pitch
                '0',  # yaw
                'legs_link',  # parent frame
                'torso_link'  # child frame
            ]
        ),

        # Static transform for the head camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_head_camera',
            arguments=[
                '0',  # x
                '0',  # y
                '0',  # z
                '0',  # roll
                '0',  # pitch
                '0',  # yaw
                'head_camera_link',  # parent frame
                'D455_head_link'  # child frame
            ]
        )
    ])
