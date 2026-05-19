/*
@brief Establishes serial connection and writes/reads from arduino-based motor controller.

@description Class is responsible for:
                - establishing serial connection between hardware running ROS 2 and arduino-based servo motor controller,
                - writing commands to motor controller such as:
                    - open
                    - close
                - reading:
                    - motor position

@author Ziga Breznikar
@date 17.05.2026
*/

#include <string>
#include <serial/serial.h>

class ArduinoComms
{
public:
    ArduinoComms()
    {}
    ArduinoComms(const std::string &serial_device, uint32_t baud_rate, uint16_t timeout)
     : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout))
    {}

    // setup comms
    void setup(const std::string &serial_device, uint32_t baud_rate, uint16_t timeout);
    // send commands
    void setPosition(const double position);
    void sendCommand(const std::string &position);
    std::string readPosition();
    // check if connection is established
    bool connected() const { return serial_conn_.isOpen(); } 
private:
    serial::Serial serial_conn_;
};
