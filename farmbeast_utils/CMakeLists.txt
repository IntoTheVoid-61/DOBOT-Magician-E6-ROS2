cmake_minimum_required(VERSION 3.8)
project(farmbeast_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dobot_msgs_fb REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(frame_transform_publisher
  src/frame_transform_publisher.cpp
)

ament_target_dependencies(frame_transform_publisher
  rclcpp
  dobot_msgs_fb
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS
  frame_transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)


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
