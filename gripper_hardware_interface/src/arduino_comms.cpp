#include "gripper_hardware_interface/arduino_comms.h"
#include <sstream>

void ArduinoComms::setup(const std::string &serial_device, uint32_t baud_rate, uint16_t timeout)
{
    serial_conn_.setPort(serial_device); 
    serial_conn_.setBaudrate(baud_rate); 
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout);
    serial_conn_.setTimeout(tt);
    serial_conn_.open();
}


void ArduinoComms::setPosition(const double position)
{
    std::stringstream ss;
    ss <<"G"<< position<<"\n";
    sendCommand(ss.str());
}

void ArduinoComms::sendCommand(const std::string &position)
{
    serial_conn_.write(position);
}

std::string ArduinoComms::readPosition()
{

    if(serial_conn_.available()){
        return serial_conn_.readline();
    }
    return "";
}