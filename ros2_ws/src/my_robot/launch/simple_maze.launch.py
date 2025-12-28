#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    pkg_my_robot = get_package_share_directory('my_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    xacro_file = os.path.join(pkg_my_robot, 'urdf', 'my_robot.urdf.xacro')
    world_file = os.path.join(pkg_my_robot, 'worlds', 'simple_maze.world')
    config_path = os.path.join(pkg_my_robot, 'config')
    
    # Process xacro file to get robot_description with config path
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' config_path:=', config_path
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    # Spawn robot inside entrance cage (x = 0.75, centered in 2.5m cage before maze entrance)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '2.25',  # Inside entrance cage
            '-y', '1.25',   # Centered
            '-z', '0.035',
            '-Y', '0.0'    # Face +X (towards maze entrance)
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
