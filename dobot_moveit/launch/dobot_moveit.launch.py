
"""
This launch file orchestrates the launching of:
    - me6_moveit/launch/dobot_moveit.launch.py
    - dobot_moveit/launch/dobot_joint.launch.py
    - dobot_system_tests/launch/fb_actions.launch.py
    - farmbeast_utils/launch/frame_transform.launch.py

"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():
    name = os.getenv("DOBOT_TYPE")
    package_name = f'{name}_moveit'
    urdf_name = "dobot_moveit.launch.py"

    pkg_share = os.path.join(get_package_share_directory(package_name))


    #--me6_moveit/launch/dobot_moveit.launch.py--#
    moveit_model_path = os.path.join(pkg_share,'launch',urdf_name)
    ld = LaunchDescription()
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_model_path)
    )
    
    #--dobot_moveit/launch/dobot_joint.launch.py
    joint_name = "dobot_joint.launch.py"
    joint_share = os.path.join(get_package_share_directory("dobot_moveit"))
    moveit_model_path = os.path.join(joint_share, 'launch', joint_name)
    joint_name_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_model_path)
    )

    #--dobot_system_tests/launch/fb_actions.launch.py--#
    launch_file_name = 'fb_actions.launch.py'
    actions_pkg_name = 'dobot_system_tests'
    action_pkg_share = get_package_share_directory(actions_pkg_name)
    actions_path = os.path.join(action_pkg_share,'launch',launch_file_name)
    fb_actions = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(actions_path)
    )
    
    #--farmbeast_utils/launch/frame_transform.launch.py--#
    launch_file_name_ft = "frame_transform.launch.py"
    utils_pkg_name = "farmbeast_utils"
    utils_pkg_share = get_package_share_directory(utils_pkg_name)
    frame_transform_path = os.path.join(utils_pkg_share,"launch",launch_file_name_ft)  
    frame_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(frame_transform_path)
    )
    

    ld.add_action(joint_name_launch)
    ld.add_action(included_launch)
    ld.add_action(fb_actions)
    ld.add_action(frame_transform)
    return ld
