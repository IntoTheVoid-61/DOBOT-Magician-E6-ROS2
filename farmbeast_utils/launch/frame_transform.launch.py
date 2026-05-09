"""
This launch file orchestrates the launching of frame_transform topic publisher.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # path specific variables
    pkg_name = "farmbeast_utils"
    pkg_share = os.path.join(get_package_share_directory(pkg_name))
    executable_name = "frame_transform_publisher"
    config_folder_name = "config"
    config_file_name = "frame_transform.yaml"
    config_path = os.path.join(pkg_share,config_folder_name,config_file_name)
    
    return LaunchDescription([
        Node(
            package=pkg_name,
            executable=executable_name,
            parameters=[config_path],
            output="screen"
        )
    ])