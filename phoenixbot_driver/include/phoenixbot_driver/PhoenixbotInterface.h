#ifndef PHOENIXBOT_INTERFACE_H
#define PHOENIXBOT_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <phoenixbot_msgs/Light.h>
#include <phoenixbot_msgs/Solenoid.h>

#include <serial/serial.h>

class PhoenixbotInterface : public hardware_interface::RobotHW {
public:
    PhoenixbotInterface(std::string port, int baud, int timeout);
    ~PhoenixbotInterface();

    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);

    void enable();
    void disable();

    phoenixbot_msgs::Light light();
    void solenoid(phoenixbot_msgs::Solenoid cmd);

private:
    serial::Serial arduino;
    hardware_interface::JointStateInterface stateInterface;
    hardware_interface::VelocityJointInterface velocityCommandInterface;
    hardware_interface::PositionJointInterface positionCommandInterface;
    joint_limits_interface::PositionJointSoftLimitsInterface positionLimitsInterface;

    double pos[5] = {0};
    double vel[5] = {0};
    double eff[5] = {0};

    double cmdVel[2] = {0};
    double cmdPos[4] = {0};

    double lightSensors[4] = {0};
    bool cmdSolenoid[4] = {false};
    int solenoidMap[4] = {5, 4, 3, 2};
};

#endif

