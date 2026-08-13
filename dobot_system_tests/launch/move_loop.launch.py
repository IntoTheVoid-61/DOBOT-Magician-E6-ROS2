"""
This launch file orchestrates the launching of move_loop
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("me6_robot", package_name="me6_moveit").to_moveit_configs()

    # path to config file
    config_file = os.path.join(get_package_share_directory("dobot_system_tests"),
                               "config","move_loop.yaml")

    move_loop = Node(
        package="dobot_system_tests",
        executable="move_loop",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            config_file
        ],
    )

    

    return LaunchDescription([
        move_loop
    ])

    