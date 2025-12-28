import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_robot_dir = get_package_share_directory('my_robot')
    rviz_config = os.path.join(my_robot_dir, 'config', 'nav2_local_view.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='my_robot',
            executable='static_map_publisher.py',
            name='static_map_publisher',
            output='screen'
        )
    ])
