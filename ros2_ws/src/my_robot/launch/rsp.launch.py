from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    urdf = f"{pkg}/urdf/my_robot.urdf.xacro"

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([FindExecutable(name='xacro'), " ", urdf])
        }],
        output='screen'
    )

    return LaunchDescription([rsp])
