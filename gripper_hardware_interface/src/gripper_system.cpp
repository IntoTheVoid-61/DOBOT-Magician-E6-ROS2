#include "gripper_hardware_interface/gripper_system.hpp"

/*
Todo: change write architecture, currently it is spamming commands
*/

using hardware_interface::CallbackReturn; 


hardware_interface::CallbackReturn gripper_arduino::GripperSystemHardware::on_init(const hardware_interface::HardwareInfo &info_)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"), "Initializing gripper hardware...");

    // 1. Load parameters
    try
    {
        const auto &p = info_.hardware_parameters;

        // serial conn specifc
        config_.serial_device = p.at("serial_device");
        config_.baud_rate = std::stoi(p.at("baud_rate"));
        config_.timeout = std::stoi(p.at("timeout"));
        config_.loop_rate = std::stof(p.at("loop_rate"));
        config_.gripper_joint_name = p.at("gripper_joint_name");

    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                    "Missing required parameter: %s", e.what());
        return CallbackReturn::ERROR;       
    }

    // 2. Allocate vectors
    hw_states.resize(2, 0.0);
    hw_commands.resize(1, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"),
                "Gripper Hardware Initialized Successfully!");
    
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> gripper_arduino::GripperSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interface;
    state_interface.emplace_back(hardware_interface::StateInterface(config_.gripper_joint_name, hardware_interface::HW_IF_POSITION, &hw_states[0]));
    state_interface.emplace_back(hardware_interface::StateInterface(config_.gripper_joint_name, hardware_interface::HW_IF_VELOCITY, &hw_states[1]));
    return state_interface;
}

std::vector<hardware_interface::CommandInterface> gripper_arduino::GripperSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interface;
    command_interface.emplace_back(hardware_interface::CommandInterface(config_.gripper_joint_name, hardware_interface::HW_IF_POSITION, &hw_commands[0]));
    return command_interface;
}

hardware_interface::CallbackReturn gripper_arduino::GripperSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"), "Connecting to Arduino...");

    try
    {
        arduino_.setup(config_.serial_device, config_.baud_rate, config_.timeout);
        // wait to stabilize connection
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                    "Error while establishing serial connection: %s", e.what());
        return CallbackReturn::ERROR;
    }

    if(!arduino_.connected())
    {
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                    "Could not establish controller connection, Arduino not responding!");
        return CallbackReturn::ERROR;        
    }

    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"),
                "Successfully connected to Arduino!");

    return CallbackReturn::SUCCESS;
    
}

hardware_interface::CallbackReturn gripper_arduino::GripperSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"), "Activating gripper...");

    if (!arduino_.connected()){
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                "Could not establish controller connection, Arduino not responding!");

        return CallbackReturn::ERROR;
    }

    // open gripper
    try
    {
        arduino_.setPosition(0.0);
        previous_command = 0.0;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                    "Failed to send command! %s", e.what());

        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"), "Hardware successfully activated!");

    return CallbackReturn::SUCCESS;
    
}


hardware_interface::CallbackReturn gripper_arduino::GripperSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"), "Deactivating gripper...");

    try
    {
        arduino_.setPosition(0.0);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                    "Failed to send command! %s", e.what());

        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("GripperHardware"), "Hardware successfully deactivated!");

    return CallbackReturn::SUCCESS;

}


hardware_interface::return_type gripper_arduino::GripperSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if (!arduino_.connected()){
            RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                "Could not establish controller connection, Arduino not responding!"); 
            return hardware_interface::return_type::ERROR;            
        }
        
        try
        {
            std::string msg = arduino_.readPosition();

            if(!msg.empty()){

                msg.erase(
                    std::remove(msg.begin(), msg.end(), '\n'),
                    msg.end()
                );

                msg.erase(
                    std::remove(msg.begin(), msg.end(), '\r'),
                    msg.end()
                );

                if (msg[0] == 'P'){
                    RCLCPP_INFO(
                        rclcpp::get_logger("GripperHardware"),
                        "received state: %s",
                        msg.c_str()
                    );
                    double pos = std::stod(msg.substr(1));

                    hw_states[0] = pos;
                }
            }
        }
        catch(const std::exception& e)
        {
        RCLCPP_ERROR(
            rclcpp::get_logger("GripperHardware"),
            "Read failed: %s",
            e.what());
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }

hardware_interface::return_type gripper_arduino::GripperSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (!arduino_.connected()){
        RCLCPP_ERROR(rclcpp::get_logger("GripperHardware"),
                "Could not establish controller connection, Arduino not responding!");

        return hardware_interface::return_type::ERROR;
    }
    
    const double gripper_command = hw_commands[0];

    if (std::abs(gripper_command - previous_command) > 0.2){

        RCLCPP_INFO(
            rclcpp::get_logger("GripperHardware"),
            "Sending gripper command: command=%f",
            gripper_command
            );
            
        arduino_.setPosition(gripper_command);
        previous_command = gripper_command;
        
    }

    return hardware_interface::return_type::OK;

}

PLUGINLIB_EXPORT_CLASS(
gripper_arduino::GripperSystemHardware, // namespace::ClassName => Must match with robot_hardware.xml
hardware_interface::SystemInterface // namespace::ClassName => Must match with robot_hardware.xml
)