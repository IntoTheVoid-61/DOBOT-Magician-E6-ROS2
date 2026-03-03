from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

"""
This Launch file handles the launching of all the necessary ros2_control files, including
controller manager with joint_state_broadcaster and joint_trajectory_controller, the necessary urdf files for resource manager and robot description.
"""

# Defining arguments from top level xacro file (dobot_e6.urdf.xacro).

ARGUMENTS = [
    DeclareLaunchArgument('robot_name',default_value="dobot_e6",
                          description='Name of the robot'),

    DeclareLaunchArgument('add_world',default_value="true",
                          choices=['true','false'],
                          description='Whether to add the world link'), 

    DeclareLaunchArgument('use_gripper',default_value="false",
                          choices=['true','false'],
                          description='Whether to use gripper'), 

    DeclareLaunchArgument('prefix',default_value="",
                          description='Prefix for robot, joints and links'),  
    
    DeclareLaunchArgument('use_mock_hardware',default_value='true',
                          choices=['true','false'],
                          description='Whether to use mock hardware plugin')

]

def generate_launch_description():
    """
    Generates the launch description for ros2_control TODO
    """

    #--Defining file and package names--#
    description_package = 'dobot_e6_description' # package containing robot description
    urdf_filename = 'dobot_e6.urdf.xacro' # top-level xacro file
    rviz_filename = 'dobot_e6_description.rviz'
    controllers_filename = 'ros2_controllers.yaml' # config file for controllers (controller_manager)
    
    #--Setting paths for importing files--#

    # Locate share directory of description_package
    pkg_share_description = FindPackageShare(description_package)

    # Build path to top level urdf file
    urdf_path = PathJoinSubstitution(
        [description_package,'urdf','robots',urdf_filename]
    )

    # Build path to rviz config file
    rviz_config_path = PathJoinSubstitution(
        [description_package,'rviz',rviz_filename]
    )

    # Build path to controllers config file
    controllers_path = PathJoinSubstitution(
        [description_package,'urdf','control',controllers_filename]
    )

    #--Declare launch file arguments--#
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='start rviz automatically'
    )

    #--Process URDF file--#
    robot_description_content = ParameterValue(Command([
        'xacro',' ',urdf_path,' ',
        'robot_name:=',LaunchConfiguration('robot_name'),' ',
        'add_world:=',LaunchConfiguration('add_world'),' ',
        'use_gripper:=',LaunchConfiguration('use_gripper'),' ',
        'prefix:=',LaunchConfiguration('prefix'),' ',
        'use_mock_hardware:=',LaunchConfiguration('use_mock_hardware'),' ',
    ]),value=str)

    #--Defining nodes--#

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_path],
        output='both'
    )

    robot_state_publisher_node = Node(
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
        arguments=['-d', rviz_config_path],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", controllers_path],
    )

    ld = LaunchDescription(ARGUMENTS)

    #--Add launch file arguments--#
    ld.add_action(declare_use_rviz_cmd)

    #--Add nodes to launch--#
    ld.add_action(control_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_rviz_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)

    return ld