import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('axis_free_rotate_ik')
    config_file = os.path.join(config, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='axis_free_rotate_ik',
            executable='axis_free_rotate_ik_ros_node',
            name='axis_free_rotate_ik',
            output='screen',
            parameters=[config_file],
        )
    ])
