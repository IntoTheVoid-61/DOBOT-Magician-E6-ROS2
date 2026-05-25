"""
This launch file orchestrates the launching of approach MTC.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("me6_robot", package_name="me6_moveit").to_moveit_configs()

    # path to config file
    config_file = os.path.join(get_package_share_directory("dobot_mtc_tasks"),
                               "config","remove_weed_params.yaml")

    remove_weed_mtc = Node(
        package="dobot_mtc_tasks",
        executable="remove_weed_mtc",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            config_file
        ],
    )

    

    return LaunchDescription([
        remove_weed_mtc
    ])

    