from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mono_depth_perception'),
        'config',
        'ground_edge_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='mono_depth_perception',
            executable='ground_edge_node',
            name='ground_edge_node',
            output='screen',
            parameters=[config]
        )
    ])
