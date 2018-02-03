#include "phoenixbot_driver/PhoenixbotInterface.h"

#include <sstream>

PhoenixbotInterface::PhoenixbotInterface(std::string port, int baud, int timeout)
    : arduino(port, baud, serial::Timeout::simpleTimeout(timeout))
{
    // Setup ros_control interfaces
    // State Handles
    hardware_interface::JointStateHandle leftStateHandle          ("left_wheel_joint",     pos + 0, vel + 0, eff + 0);
    hardware_interface::JointStateHandle rightStateHandle         ("right_wheel_joint",    pos + 1, vel + 1, eff + 1);
    hardware_interface::JointStateHandle gripperLiftStateHandle   ("gripper_lift_joint",   pos + 2, vel + 2, eff + 2);
    hardware_interface::JointStateHandle mobileGripperStateHandle ("mobile_gripper_joint", pos + 3, vel + 3, eff + 3);
    hardware_interface::JointStateHandle staticGripperStateHandle ("static_gripper_joint", pos + 4, vel + 4, eff + 4);
    hardware_interface::JointStateHandle simonXAxisStateHandle    ("simon_x_axis_joint",   pos + 5, vel + 5, eff + 5);
    hardware_interface::JointStateHandle simonYAxisStateHandle    ("simon_y_axis_joint",   pos + 6, vel + 6, eff + 6);

    // Register state handles
    stateInterface.registerHandle(leftStateHandle);
    stateInterface.registerHandle(rightStateHandle);
    stateInterface.registerHandle(gripperLiftStateHandle);
    stateInterface.registerHandle(mobileGripperStateHandle);
    stateInterface.registerHandle(staticGripperStateHandle);
    stateInterface.registerHandle(simonXAxisStateHandle);
    stateInterface.registerHandle(simonYAxisStateHandle);

    // Drive handles
    hardware_interface::JointHandle leftVelocityHandle  (leftStateHandle,  cmdVel + 0);
    hardware_interface::JointHandle rightVelocityHandle (rightStateHandle, cmdVel + 1);

    // Register drive handles
    velocityCommandInterface.registerHandle(leftVelocityHandle);
    velocityCommandInterface.registerHandle(rightVelocityHandle);

    // Stepper handles
    hardware_interface::JointHandle gripperLiftPositionHandle   (gripperLiftStateHandle,   cmdPos + 0);
    hardware_interface::JointHandle mobileGripperPositionHandle (mobileGripperStateHandle, cmdPos + 1);
    hardware_interface::JointHandle staticGripperPositionHandle (staticGripperStateHandle, cmdPos + 2);
    hardware_interface::JointHandle simonXAxisPositionHandle    (simonXAxisStateHandle,    cmdPos + 3);
    hardware_interface::JointHandle simonYAxisPositionHandle    (simonYAxisStateHandle,    cmdPos + 4);

    // Register stepper handles
    positionCommandInterface.registerHandle(gripperLiftPositionHandle);
    positionCommandInterface.registerHandle(mobileGripperPositionHandle);
    positionCommandInterface.registerHandle(staticGripperPositionHandle);
    positionCommandInterface.registerHandle(simonXAxisPositionHandle);
    positionCommandInterface.registerHandle(simonYAxisPositionHandle);

    registerInterface(&stateInterface);
    registerInterface(&velocityCommandInterface);
    registerInterface(&positionCommandInterface);

    // TODO Initalize communication with arduino
}

// TODO End communication and tear down
PhoenixbotInterface::~PhoenixbotInterface() {
}

// Read the state of everything from the robot
void PhoenixbotInterface::read() {
    // TODO Read current steps of all stepper motors and convert to radiens
    // TODO Read current encoder position of drive motors
    // TODO Calculate velocity of drive motors
}

// Push commands to the robot
void PhoenixbotInterface::write() {
    std::stringstream serialString;

    serialString << "M L " << (int)(cmdVel[0] * 500) << "\r";
    arduino.write(serialString.str());

    serialString << "M R " << (int)(cmdVel[1] * 500) << "\r";
    arduino.write(serialString.str());

    // TODO Convert from radiens to steps
    // TODO Send step targets to stepper motors
    // TODO Send velocity targets to drive motors
}

