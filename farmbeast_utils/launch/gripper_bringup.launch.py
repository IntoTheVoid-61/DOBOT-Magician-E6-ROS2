from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("dobot_e6_description"),'config','gripper_controllers.yaml']
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster"
        ],
        output = "screen"
    )

    # ros2_control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="both",
    )    

    # Gripper Controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--param-file",
            controllers_file,
        ],
    )  

    ld = LaunchDescription()

    ld.add_action(control_node)  
    ld.add_action(
        TimerAction(
            period = 3.0,
            actions = [joint_state_broadcaster]
        )
    )

    ld.add_action(
        TimerAction(
            period = 3.0,
            actions = [robot_controller_spawner]
        )
    )

    return ld
