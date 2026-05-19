cmake_minimum_required(VERSION 3.8)
project(gripper_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#--- Vendor serial library ---#
add_subdirectory(serial_lib)

#--- Create plugin library ---#
add_library(gripper_hardware_interface SHARED src/arduino_comms.cpp src/gripper_system.cpp)

# THIS FIXES THE LINKER ERROR
set_target_properties(serial PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Include directories
target_include_directories(
  gripper_hardware_interface
  PRIVATE
  include
  serial_lib/include
)

# Link dependencies
ament_target_dependencies(
  gripper_hardware_interface
  hardware_interface
  rclcpp
  pluginlib
)

# Link vendored library
target_link_libraries(gripper_hardware_interface serial)


pluginlib_export_plugin_description_file(hardware_interface gripper_hardware.xml)

install(
  TARGETS gripper_hardware_interface
  DESTINATION lib
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