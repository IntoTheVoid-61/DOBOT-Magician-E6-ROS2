cmake_minimum_required(VERSION 3.8)
project(dobot_msgs_fb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# MoveToPose, depends on geometry_msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "action/MoveToHome.action"
  "msg/FrameTransform.msg"
  "srv/ApproachObject.srv"
  "srv/RemoveWeed.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# MoveToHome, no dependencies
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "action/MoveToHome.action"
#)

# FrameTransform, depends on geometry_msgs and builtin_interfaces
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/FrameTransform.msg"
#  DEPENDENCIES geometry_msgs builtin_interfaces
#)

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
