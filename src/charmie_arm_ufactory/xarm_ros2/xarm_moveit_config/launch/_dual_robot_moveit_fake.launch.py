#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_realsense_d435i_1 = LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2 = LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)

    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_d435i_links_1 = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2 = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
    
    model1300 = LaunchConfiguration('model1300', default=False)
    model1300_1 = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2 = LaunchConfiguration('model1300_2', default=model1300)

    robot_sn = LaunchConfiguration('robot_sn', default='')
    robot_sn_1 = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2 = LaunchConfiguration('robot_sn_2', default=robot_sn)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    add_other_geometry_1 = LaunchConfiguration('add_other_geometry_1', default=add_other_geometry)
    add_other_geometry_2 = LaunchConfiguration('add_other_geometry_2', default=add_other_geometry)

    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_type_1 = LaunchConfiguration('geometry_type_1', default=geometry_type)
    geometry_type_2 = LaunchConfiguration('geometry_type_2', default=geometry_type)

    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_mass_1 = LaunchConfiguration('geometry_mass_1', default=geometry_mass)
    geometry_mass_2 = LaunchConfiguration('geometry_mass_2', default=geometry_mass)

    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_height_1 = LaunchConfiguration('geometry_height_1', default=geometry_height)
    geometry_height_2 = LaunchConfiguration('geometry_height_2', default=geometry_height)

    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_radius_1 = LaunchConfiguration('geometry_radius_1', default=geometry_radius)
    geometry_radius_2 = LaunchConfiguration('geometry_radius_2', default=geometry_radius)
    
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_length_1 = LaunchConfiguration('geometry_length_1', default=geometry_length)
    geometry_length_2 = LaunchConfiguration('geometry_length_2', default=geometry_length)

    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_width_1 = LaunchConfiguration('geometry_width_1', default=geometry_width)
    geometry_width_2 = LaunchConfiguration('geometry_width_2', default=geometry_width)

    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_filename_1 = LaunchConfiguration('geometry_mesh_filename_1', default=geometry_mesh_filename)
    geometry_mesh_filename_2 = LaunchConfiguration('geometry_mesh_filename_2', default=geometry_mesh_filename)
    
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_xyz_1 = LaunchConfiguration('geometry_mesh_origin_xyz_1', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_2 = LaunchConfiguration('geometry_mesh_origin_xyz_2', default=geometry_mesh_origin_xyz)
    
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_origin_rpy_1 = LaunchConfiguration('geometry_mesh_origin_rpy_1', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_2 = LaunchConfiguration('geometry_mesh_origin_rpy_2', default=geometry_mesh_origin_rpy)
    
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_xyz_1 = LaunchConfiguration('geometry_mesh_tcp_xyz_1', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_2 = LaunchConfiguration('geometry_mesh_tcp_xyz_2', default=geometry_mesh_tcp_xyz)
    
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    geometry_mesh_tcp_rpy_1 = LaunchConfiguration('geometry_mesh_tcp_rpy_1', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_2 = LaunchConfiguration('geometry_mesh_tcp_rpy_2', default=geometry_mesh_tcp_rpy)

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1 = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2 = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
    controllers_name = 'fake_controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    
    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    xacro_file = PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'dual_xarm_device.urdf.xacro'])
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=xacro_file, 
            arguments={
                'prefix_1': prefix_1,
                'prefix_2': prefix_2,
                'dof_1': dof_1,
                'dof_2': dof_2,
                'robot_type_1': robot_type_1,
                'robot_type_2': robot_type_2,
                'add_gripper_1': add_gripper_1,
                'add_gripper_2': add_gripper_2,
                'add_vacuum_gripper_1': add_vacuum_gripper_1,
                'add_vacuum_gripper_2': add_vacuum_gripper_2,
                'add_bio_gripper_1': add_bio_gripper_1,
                'add_bio_gripper_2': add_bio_gripper_2,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'add_realsense_d435i_1': add_realsense_d435i_1,
                'add_realsense_d435i_2': add_realsense_d435i_2,
                'add_d435i_links_1': add_d435i_links_1,
                'add_d435i_links_2': add_d435i_links_2,
                'model1300_1': model1300_1,
                'model1300_2': model1300_2,
                'robot_sn_1': robot_sn_1,
                'robot_sn_2': robot_sn_2,
                'add_other_geometry_1': add_other_geometry_1,
                'add_other_geometry_2': add_other_geometry_2,
                'geometry_type_1': geometry_type_1,
                'geometry_type_2': geometry_type_2,
                'geometry_mass_1': geometry_mass_1,
                'geometry_mass_2': geometry_mass_2,
                'geometry_height_1': geometry_height_1,
                'geometry_height_2': geometry_height_2,
                'geometry_radius_1': geometry_radius_1,
                'geometry_radius_2': geometry_radius_2,
                'geometry_length_1': geometry_length_1,
                'geometry_length_2': geometry_length_2,
                'geometry_width_1': geometry_width_1,
                'geometry_width_2': geometry_width_2,
                'geometry_mesh_filename_1': geometry_mesh_filename_1,
                'geometry_mesh_filename_2': geometry_mesh_filename_2,
                'geometry_mesh_origin_xyz_1': geometry_mesh_origin_xyz_1,
                'geometry_mesh_origin_xyz_2': geometry_mesh_origin_xyz_2,
                'geometry_mesh_origin_rpy_1': geometry_mesh_origin_rpy_1,
                'geometry_mesh_origin_rpy_2': geometry_mesh_origin_rpy_2,
                'geometry_mesh_tcp_xyz_1': geometry_mesh_tcp_xyz_1,
                'geometry_mesh_tcp_xyz_2': geometry_mesh_tcp_xyz_2,
                'geometry_mesh_tcp_rpy_1': geometry_mesh_tcp_rpy_1,
                'geometry_mesh_tcp_rpy_2': geometry_mesh_tcp_rpy_2,
                'kinematics_suffix_1': kinematics_suffix_1,
                'kinematics_suffix_2': kinematics_suffix_2,
            }
        )
    }

    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # robot moveit common launch
    # xarm_moveit_config/launch/_dual_robot_moveit_common.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_common.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            # 'add_gripper_1': add_gripper_1 if robot_type_1.perform(context) == 'xarm' else 'false',
            # 'add_gripper_2': add_gripper_2 if robot_type_2.perform(context) == 'xarm' else 'false',
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'no_gui_ctrl': no_gui_ctrl,
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value,
            'add_realsense_d435i_1': add_realsense_d435i_1,
            'add_realsense_d435i_2': add_realsense_d435i_2,
            'add_d435i_links_1': add_d435i_links_1,
            'add_d435i_links_2': add_d435i_links_2,
            'model1300_1': model1300_1,
            'model1300_2': model1300_2,
            'robot_sn_1': robot_sn_1,
            'robot_sn_2': robot_sn_2,
            'add_other_geometry_1': add_other_geometry_1,
            'add_other_geometry_2': add_other_geometry_2,
            'geometry_type_1': geometry_type_1,
            'geometry_type_2': geometry_type_2,
            'geometry_mass_1': geometry_mass_1,
            'geometry_mass_2': geometry_mass_2,
            'geometry_height_1': geometry_height_1,
            'geometry_height_2': geometry_height_2,
            'geometry_radius_1': geometry_radius_1,
            'geometry_radius_2': geometry_radius_2,
            'geometry_length_1': geometry_length_1,
            'geometry_length_2': geometry_length_2,
            'geometry_width_1': geometry_width_1,
            'geometry_width_2': geometry_width_2,
            'geometry_mesh_filename_1': geometry_mesh_filename_1,
            'geometry_mesh_filename_2': geometry_mesh_filename_2,
            'geometry_mesh_origin_xyz_1': geometry_mesh_origin_xyz_1,
            'geometry_mesh_origin_xyz_2': geometry_mesh_origin_xyz_2,
            'geometry_mesh_origin_rpy_1': geometry_mesh_origin_rpy_1,
            'geometry_mesh_origin_rpy_2': geometry_mesh_origin_rpy_2,
            'geometry_mesh_tcp_xyz_1': geometry_mesh_tcp_xyz_1,
            'geometry_mesh_tcp_xyz_2': geometry_mesh_tcp_xyz_2,
            'geometry_mesh_tcp_rpy_1': geometry_mesh_tcp_rpy_1,
            'geometry_mesh_tcp_rpy_2': geometry_mesh_tcp_rpy_2,
            'kinematics_suffix_1': kinematics_suffix_1,
            'kinematics_suffix_2': kinematics_suffix_2,
        }.items(),
    )

    remappings = [
        ('follow_joint_trajectory', '{}{}{}_traj_controller/follow_joint_trajectory'.format(prefix_1.perform(context), robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')),
        ('follow_joint_trajectory', '{}{}{}_traj_controller/follow_joint_trajectory'.format(prefix_2.perform(context), robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')),            
    ]
    controllers = [
        '{}{}{}_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else ''),
        '{}{}{}_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else ''),
    ]
    if add_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        remappings.append(
            ('follow_joint_trajectory', '{}{}_gripper_traj_controller/follow_joint_trajectory'.format(prefix_1.perform(context), robot_type_1.perform(context)))
        )
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context)))
    elif add_bio_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        remappings.append(
            ('follow_joint_trajectory', '{}bio_gripper_traj_controller/follow_joint_trajectory'.format(prefix_1.perform(context)))
        )
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_1.perform(context)))
    if add_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        remappings.append(
            ('follow_joint_trajectory', '{}{}_gripper_traj_controller/follow_joint_trajectory'.format(prefix_2.perform(context), robot_type_2.perform(context)))
        )
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context)))
    elif add_bio_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        remappings.append(
            ('follow_joint_trajectory', '{}bio_gripper_traj_controller/follow_joint_trajectory'.format(prefix_2.perform(context)))
        )
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_2.perform(context)))
    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['joint_states']}],
        remappings=remappings,
    )

    # ros2 control launch
    # xarm_controller/launch/_dual_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_dual_ros2_control.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'ros2_control_plugin': ros2_control_plugin,
            'add_realsense_d435i_1': add_realsense_d435i_1,
            'add_realsense_d435i_2': add_realsense_d435i_2,
            'add_d435i_links_1': add_d435i_links_1,
            'add_d435i_links_2': add_d435i_links_2,
            'model1300_1': model1300_1,
            'model1300_2': model1300_2,
            'robot_sn_1': robot_sn_1,
            'robot_sn_2': robot_sn_2,
            'kinematics_suffix_1': kinematics_suffix_1,
            'kinematics_suffix_2': kinematics_suffix_2,
        }.items(),
    )

    # Load controllers
    load_controllers = []
    for controller in controllers:
        load_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))


    return [
        robot_state_publisher_node,
        robot_moveit_common_launch,
        joint_state_publisher_node,
        ros2_control_launch,
    ] + load_controllers


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
