#ifndef PHOENIXBOT_INTERFACE_H
#define PHOENIXBOT_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <phoenixbot_msgs/Light.h>
#include <phoenixbot_msgs/Solenoid.h>

#include <serial/serial.h>

class PhoenixbotInterface : public hardware_interface::RobotHW {
public:
    PhoenixbotInterface(std::string port, int baud, int timeout);
    ~PhoenixbotInterface();

    void read();
    void write();

    void enable();
    void disable();

    phoenixbot_msgs::Light light();
    void solenoid(phoenixbot_msgs::Solenoid cmd);

private:
    serial::Serial arduino;
    hardware_interface::JointStateInterface stateInterface;
    hardware_interface::VelocityJointInterface velocityCommandInterface;
    hardware_interface::PositionJointInterface positionCommandInterface;

    double pos[4];
    double vel[4];
    double eff[4];

    double cmdVel[4] = {0};

    double lightSensors[4];
    bool cmdSolenoid[4] = {false};
    int solenoidMap[4] = {5, 4, 3, 2};
};

#endif

