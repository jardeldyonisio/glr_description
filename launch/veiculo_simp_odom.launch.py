#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import launch
import launch_ros
import launch_ros.parameter_descriptions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    urdf_file_path = 'src/description/vehicle_description_bigga_parent_control.urdf'

    pkg_share = launch_ros.substitutions.FindPackageShare(package='freedom_vehicle').find('freedom_vehicle')
    default_model_path = os.path.join(pkg_share, 'src/description/vehicle_description_bigga_parent_control.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
        
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_freedom_gazebo = get_package_share_directory('freedom_vehicle')
        
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ###############
    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "freedom_vehicle"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_freedom_gazebo , 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'
    ##########

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.33'
    spawn_roll_val = '0.0'
    spawn_pitch_val = '0.0'
    spawn_yaw_val = '0.0'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'pause': 'false', 'use_sim_time': 'true'}.items()
    )

    controle_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        name='controller_manager',
        parameters=[os.path.join(pkg_share, 'config', 'controllers.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
       
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('urdf_model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('joint_gui')),
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('joint_gui')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spwan_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'freedomvehicle', 
                   '-x', spawn_x_val, 
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-R', spawn_roll_val,
                   '-P', spawn_pitch_val, 
                   '-Y', spawn_yaw_val,],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'controllers.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
            
    eixorodavolantebigga_steer_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["eixorodavolantebigga_steer_controller", "-c", "/controller_manager"],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'controllers.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    eixorodatras1bigga_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["eixorodatras1bigga_controller", "-c", "/controller_manager"],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'controllers.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    eixorodatras2bigga_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["eixorodatras2bigga_controller", "-c", "/controller_manager"],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'controllers.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
       
    
    
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"]
    )
    eixorodavolantebigga_steer_controller_spawner = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "eixorodavolantebigga_steer_controller"]
    )
    eixorodatras1bigga_controller_spawner = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "eixorodatras1bigga_controller"]
    )
    eixorodatras2bigga_controller_spawner = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "eixorodatras2bigga_controller"]
    )

    
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value=[os.path.join(pkg_freedom_gazebo, 'worlds', 'empty.world'), ''],
            #empty.world
            description='Full path to the world model file to load'),
        DeclareLaunchArgument(
            name='joint_gui', 
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'),
        DeclareLaunchArgument(
            name='urdf_model', 
            default_value=default_urdf_model_path, 
            description='Absolute path to robot urdf file'),
        gazebo,   
        controle_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_broadcaster_spawner,
        eixorodavolantebigga_steer_controller_spawner,
        eixorodatras1bigga_controller_spawner,
        eixorodatras2bigga_controller_spawner,
        rviz_node,   
        spawn_entity,
    ])
