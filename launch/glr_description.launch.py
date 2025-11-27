#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true')

    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('glr_description'),
            'urdf',
            'glr.xacro'
        ]),
        description='Full path to urdf file to load')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('glr_description'),
            'rviz',
            'display.rviz'
        ]),
        description='Full path to the RVIZ config file to use')

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_urdf_file_cmd,
        declare_rviz_config_file_cmd,
        
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])