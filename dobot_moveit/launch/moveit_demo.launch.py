"""
@brief Official dobot launch file for moveit demo, also including:
    - move_to_pose_action server

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
    urdf_name = "demo.launch.py"

    pkg_share = os.path.join(get_package_share_directory(package_name))

    moveit_model_path = os.path.join(pkg_share,'launch',urdf_name)
    ld = LaunchDescription()
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_model_path)
    )

    move_to_pose_action_server_node = Node(
        package='dobot_system_tests',
        executable='move_to_pose_action_server',
        output='screen'
    )


    ld.add_action(included_launch)
    ld.add_action(move_to_pose_action_server_node)

    # 添加其他操作...
    return ld