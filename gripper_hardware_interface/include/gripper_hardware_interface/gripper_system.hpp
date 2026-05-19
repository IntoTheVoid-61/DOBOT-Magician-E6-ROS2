/*
@brief Custom hardware plugin for gripper.

@description

on_init():
    Reads and saves hardware parameters into Config struct

export_state_interfaces():
    Defines what feedback values the hardware can provide.
    Returns a list (single in our case) of pointers to the variables in hardware class that store state values.

export_command_interfaces():
    Defines what command values the hardware can accept.
    Returns a list (single in our case) of pointers to the variables in hardware class that store command values.

on_configure():
    Used to establish a serial connection with Arduino

on_activate():
    Opens gripper

on_deactivate():
    Opens gripper

@author Ziga Breznikar
@date 17.05.2026
*/

#include <memory>
#include <string>
#include <chrono>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include "pluginlib/class_list_macros.hpp"

#include "config.h"
#include "arduino_comms.h"

namespace gripper_arduino
{
class GripperSystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(GripperSystemHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info_) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // to be decided how to implement based on arduino side code
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


private:
    Config config_;
    ArduinoComms arduino_;
    std::vector<double> hw_states; // Stores data about positional state of gripper joint
    std::vector<double> hw_commands; // Stores command data
    double previous_command; // compares absolute diff with current command
}; // GripperSystemHardware
} // namespace gripper_arduino