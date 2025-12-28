#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_mono_depth = get_package_share_directory('mono_depth_perception')
    pkg_my_robot = get_package_share_directory('my_robot')
    
    # Ground edge detection launch
    ground_edge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mono_depth, 'launch', 'ground_edge.launch.py')
        )
    )
    
    # Nav2 local navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot, 'launch', 'nav2_local.launch.py')
        )
    )

    return LaunchDescription([
        ground_edge_launch,
        nav2_launch,
    ])
