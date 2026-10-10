import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory('handle_arm'),
        'config',
        'handle_arm_trajectory_loop.yaml',
    )
    default_trajectory_file = os.path.join(
        get_package_share_directory('handle_arm'),
        'config',
        'trajectory.yaml',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the handle_arm trajectory loop parameter YAML file',
    )
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory_file,
        description='Path to the trajectory YAML file',
    )

    return LaunchDescription([
        config_file_arg,
        trajectory_file_arg,
        Node(
            package='handle_arm',
            executable='handle_arm_trajectory_loop_node',
            name='handle_arm_trajectory_loop',
            parameters=[
                LaunchConfiguration('config_file'),
                {'trajectory_file': LaunchConfiguration('trajectory_file')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
