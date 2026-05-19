"""
Launch file is used to bringup and test the hardware interface for arduino based gripper control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    # Define file names
    urdf_package = "gripper_system_tests" # package containing gripper description and config files
    urdf_filename = "gripper.urdf.xacro"
    rviz_config_filename = "gripper_description.rviz"
    controllers_filename = 'gripper_controllers.yaml'

    # Find the package housing all the files above
    pkg_share_description = FindPackageShare(urdf_package)

    urdf_model_path = PathJoinSubstitution(
        [pkg_share_description,'urdf','gripper',urdf_filename]
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_share_description,'rviz', rviz_config_filename]
    )

    robot_controllers_path = PathJoinSubstitution(
        [pkg_share_description,'config',controllers_filename]
    )

    use_rviz = LaunchConfiguration('use_rviz') # Arg about whether or not to use rviz
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Start up RViz automatically'
    )

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_model_path,
        description='Absolute path to robot urdf file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    # Generate the description parameter by processing xacro file
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=gripper', ' ',
    ]), value_type=str)

    #--Defining nodes--#
    # Defining which Nodes (programs) to start with launch file


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_path],
        output="both",
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]                
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--param-file",
            robot_controllers_path,
        ],
    )

    ld = LaunchDescription()


    # Add command line launch arguments (DeclareLaunchArguments)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add nodes to launch
    ld.add_action(control_node)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz_cmd)
    ld.add_action(TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]))
    ld.add_action(TimerAction(period=5.0, actions=[robot_controller_spawner]))

    return ld