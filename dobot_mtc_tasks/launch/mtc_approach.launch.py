"""
This launch file orchestrates the launching of approach MTC.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("me6_robot", package_name="me6_moveit").to_moveit_configs()

    approach_mtc = Node(
        package="dobot_mtc_tasks",
        executable="approach_mtc",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    

    return LaunchDescription([
        approach_mtc,
    ])

    