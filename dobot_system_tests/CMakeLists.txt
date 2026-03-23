cmake_minimum_required(VERSION 3.8)
project(dobot_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dobot_msgs_fb REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# install directories
install (
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# add move_to_pose_action_server executable
add_executable(move_to_pose_action_server
  src/move_to_pose_action_server.cpp
)

# add move_to_pose_action_client executable
add_executable(move_to_pose_action_client
  src/move_to_pose_action_client.cpp)

# add move_to_home_action_server executable
add_executable(move_to_home_action_server
  src/move_to_home_action_server.cpp)  

# add move_to_home_action_client executable
add_executable(move_to_home_action_client
  src/move_to_home_action_client.cpp)

# specify dependencies for move_to_pose_action_server
ament_target_dependencies(move_to_pose_action_server
  rclcpp
  rclcpp_action
  dobot_msgs_fb
  moveit_ros_planning_interface
)

# specify the dependencies for move_to_pose_action_client
ament_target_dependencies(move_to_pose_action_client
  rclcpp
  rclcpp_action
  dobot_msgs_fb
  geometry_msgs
)

# specify dependencies for move_to_home_action_server
ament_target_dependencies(move_to_home_action_server
  rclcpp
  rclcpp_action
  dobot_msgs_fb
  moveit_ros_planning_interface
)

# specify dependencies for move_to_home_action_client
ament_target_dependencies(move_to_home_action_client
  rclcpp
  rclcpp_action
  dobot_msgs_fb
  geometry_msgs
)


# Install the executables move_to_pose_action_server
install(TARGETS
  move_to_pose_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executables move_to_pose_action_client
install(TARGETS
  move_to_pose_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executables move_to_pose_action_server
install(TARGETS
  move_to_home_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executables move_to_pose_action_client
install(TARGETS
  move_to_home_action_client
  DESTINATION lib/${PROJECT_NAME}
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
