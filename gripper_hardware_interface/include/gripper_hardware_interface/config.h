/*
@brief Parameters defined in struct will be read by on_init() function.

@description Configuration parameters for hardware interface, replaces a yaml file. Parameters defined in the
following struct are also specified in ros2_control urdf file.

@author Ziga Breznikar
@date 17.05.2026
*/

#include <string>
#include <cstdint>

struct Config
{
    // Arduino specific configs
    std::string serial_device;
    uint32_t baud_rate;
    uint16_t timeout; // in ms
    float loop_rate;
    std::string gripper_joint_name;
    // put others here...
};
