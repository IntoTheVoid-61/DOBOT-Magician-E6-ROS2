"""
This launch file orchestrates the launching of all FarmBeast
related actions. 

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # path specific variables
    actions_pkg_name = 'dobot_system_tests'
    actions_pkg_share = os.path.join(get_package_share_directory(actions_pkg_name))
    config_folder_name = 'config'
    move_to_home_config_file_name = "move_to_home.yaml"
    move_to_home_config = os.path.join(actions_pkg_share,config_folder_name,move_to_home_config_file_name)

    
    return LaunchDescription([
        Node(
            package=actions_pkg_name,
            executable='move_to_pose_action_server',
            output='screen'
        ),

        Node(
            package=actions_pkg_name,
            executable='move_to_home_action_server',
            parameters=[move_to_home_config],
            output='screen'
        )
    ])


    return ld






