#ifndef PHOENIXBOT_INTERFACE_H
#define PHOENIXBOT_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <serial/serial.h>

class PhoenixbotInterface : public hardware_interface::RobotHW {
public:
    PhoenixbotInterface(std::string port, int baud, int timeout);
    ~PhoenixbotInterface();

    void read();
    void write();

private:
    serial::Serial arduino;
    hardware_interface::JointStateInterface stateInterface;
    hardware_interface::VelocityJointInterface velocityCommandInterface;
    hardware_interface::PositionJointInterface positionCommandInterface;

    double pos[7];
    double vel[7];
    double eff[7];

    double cmdVel[2];
    double cmdPos[5];
};

#endif

