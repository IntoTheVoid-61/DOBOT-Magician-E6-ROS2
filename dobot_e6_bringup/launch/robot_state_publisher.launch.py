from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

""" 
This Launch file handles the launching of RVIZ visualization of the mobile robot.
It configures the robot state publisher and handles the processing of the URDF/XACRO files.
"""



# Defining arguments from top level xacro file (mycobot_280.urdf.xacro). These are static.

ARGUMENTS = [
    DeclareLaunchArgument('robot_name',default_value="dobot_e6",
                          description='Name of the robot'),

    DeclareLaunchArgument('add_world',default_value="true",
                          choices=['true','false'],
                          description='Whether to add the world link'), 

    DeclareLaunchArgument('prefix',default_value="",
                          description='Prefix for robot, joints and links'),  

    DeclareLaunchArgument('use_gripper',default_value="false",
                          choices=['true','false'],
                          description='Whether to use gripper'),     
]


def generate_launch_description():
    """ 
    Generates the launch description for robot visualization in RViz or simulation.
    This file launches:
        - The robot_state_publisher node (to publish TF and /robot_description)


    """

    # Define file names
    urdf_package = 'dobot_e6_description' # package containing robot description
    urdf_filename = 'dobot_e6.urdf.xacro' # top-level xacro file
    rviz_filename = 'dobot_e6_description.rviz'

    #---Setting paths for importing files---#

    # Use FindPackageShare to locate the package at runtime
    pkg_share_description = FindPackageShare(urdf_package)

    # Building the path to top-level xacro file
    urdf_model_path = PathJoinSubstitution(
        [pkg_share_description,'urdf','robots',urdf_filename]
    )

    # Building the path to rviz config file
    rviz_config_path= PathJoinSubstitution(
        [pkg_share_description,'rviz',rviz_filename]
    )

    #--Define dynamic launch configurations (values can be changed at runtime)--#
    # Each declaration of LaunchConfiguration is followed by DeclareLaunchArgument, where we declare the names (equivalent as in LaunchConfiguration) and define default and possible values
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_jsp = LaunchConfiguration('use_jsp')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    #--Declare command-line arguments--#
    # Add them in command line launch arguments (DeclareLaunchArguments) .add_action() section

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=urdf_model_path,
        description='Absolute path to robot urdf file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )
    
    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='true',
        choices=['true','false'],
        description='Enable the joint state publisher'
    )

    declare_use_jsp_gui_cmd = DeclareLaunchArgument(
        name='use_jsp_gui',
        default_value='true',
        choices=['true','false'],
        description='Enable jsp gui'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        choices=['true','false'],
        description='Launch rviz'
    )
    
    # Generate the description parameter by processing xacro file
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
    ]), value_type=str)

    #--Defining nodes--#

    # Node publishes the TF tree and the /robot_description parameter to the ROS2 system

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
                     
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        #parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp))
    

    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        #parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp_gui))
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        #parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    

    # Create launch description and populate with arguments
    ld = LaunchDescription(ARGUMENTS)

    # Add command line launch arguments (DeclareLaunchArguments)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_jsp_cmd) # jsp is needed if joints are not fixed
    ld.add_action(declare_use_jsp_gui_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add nodes to launch (Node)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_rviz_cmd)


    return ld