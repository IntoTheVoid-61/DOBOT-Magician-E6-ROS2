cmake_minimum_required(VERSION 3.8)
project(dobot_mtc_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dobot_msgs_fb REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# install directories
install (
  DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

# add approach_mtc executable
add_executable(approach_mtc src/approach_mtc.cpp)

# add approach executable
add_executable(remove_weed_mtc src/remove_weed_mtc.cpp)

# add executable
add_executable(avoid_remove_weed_mtc src/avoid_remove_weed_mtc.cpp)

# include directory for avoid_remove_weed_mtc
target_include_directories(avoid_remove_weed_mtc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# specify dependencies for approach_mtc
ament_target_dependencies(
  approach_mtc
  moveit_task_constructor_core
  rclcpp
)

# specify dependencies for remove_weed_mtc
ament_target_dependencies(
  remove_weed_mtc
  moveit_task_constructor_core
  rclcpp
  dobot_msgs_fb
)

# specify dependencies for remove_weed_mtc
ament_target_dependencies(
  avoid_remove_weed_mtc
  moveit_task_constructor_core
  rclcpp
)

# Install the executable for approach_mtc
install(TARGETS
  approach_mtc remove_weed_mtc avoid_remove_weed_mtc
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
