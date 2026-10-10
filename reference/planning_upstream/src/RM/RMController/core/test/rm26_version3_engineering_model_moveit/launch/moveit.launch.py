from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription
import yaml
import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
import os
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def get_moveit_launch_config(ld : LaunchDescription, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    launch_move_group = generate_move_group_launch(moveit_config)

    for action in launch_move_group.entities:
        ld.add_action(action)

    # Run Rviz and load the default config to see the state of the move_group node
    launch_rviz = generate_moveit_rviz_launch(moveit_config)

    for action in launch_rviz.entities:
        ld.add_action(action)

def generate_launch_description():
    robot_description_pkg_dir = get_package_share_directory('rm26_version3_engineering_model_moveit')

    # add mock enable parameter
    ld = LaunchDescription()

    moveit_config = (MoveItConfigsBuilder("rm26_version3_engineering_model", package_name="rm26_version3_engineering_model_moveit")
                     .robot_description(
                         file_path=os.path.join(robot_description_pkg_dir, "config", "rm26_version3_engineering_model.urdf.xacro"),
                         mappings={"use_mock_hardware":  "false"})
                     .robot_description_semantic(
                         file_path=os.path.join(robot_description_pkg_dir, "config", "rm26_version3_engineering_model.srdf")
                     )
                     .to_moveit_configs())
    
    
    
    # add control.launch.py as a launch description
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_pkg_dir, "launch", "control.launch.py")
            )
        )
    )

    get_moveit_launch_config(ld, moveit_config)


    return ld