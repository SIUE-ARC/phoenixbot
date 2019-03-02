#include "phoenixbot_driver/PhoenixbotInterface.h"

#include <sstream>

#define PI 3.14159

#define TICKS_TO_RAD       ((PI * 2.0) / (1440 * 4.0))
#define TICKS_TO_RAD_SIMON ((PI * 2.0) / (7 * 4.0))

#define MICROS_TO_RAD ((PI * 2.0) / (500))

PhoenixbotInterface::PhoenixbotInterface(std::string port, int baud, int timeout)
    : arduino(port, baud, serial::Timeout::simpleTimeout(timeout))
{
    // Setup ros_control interfaces
    // State Handles
    hardware_interface::JointStateHandle leftStateHandle  ("left_wheel_joint",   pos + 0, vel + 0, eff + 0);
    hardware_interface::JointStateHandle rightStateHandle ("right_wheel_joint",  pos + 1, vel + 1, eff + 1);
    hardware_interface::JointStateHandle simonArmState    ("simon_arm_joint",    pos + 2, vel + 2, eff + 2);
    hardware_interface::JointStateHandle leftPaddleState ("left_paddle_joint",   pos + 3, vel + 3, eff + 3);
    hardware_interface::JointStateHandle rightPaddleState ("right_paddle_joint", pos + 4, vel + 4, eff + 4);

    // Register state handles
    stateInterface.registerHandle(leftStateHandle);
    stateInterface.registerHandle(rightStateHandle);
    stateInterface.registerHandle(simonArmState);
    stateInterface.registerHandle(leftPaddleState);
    stateInterface.registerHandle(rightPaddleState);

    // Register interfaces
    registerInterface(&stateInterface);

    // Drive handles
    hardware_interface::JointHandle leftVelocityHandle  (leftStateHandle,  cmdVel + 0);
    hardware_interface::JointHandle rightVelocityHandle (rightStateHandle, cmdVel + 1);

    velocityCommandInterface.registerHandle(leftVelocityHandle);
    velocityCommandInterface.registerHandle(rightVelocityHandle);
    registerInterface(&velocityCommandInterface);

    // Position
    hardware_interface::JointHandle simonArmPosition    (simonArmState, cmdPos + 3);
    hardware_interface::JointHandle leftPaddlePosition  (leftPaddleState,  cmdPos + 1);
    hardware_interface::JointHandle rightPaddlePosition (rightPaddleState, cmdPos + 2);

    positionCommandInterface.registerHandle(simonArmPosition);
    positionCommandInterface.registerHandle(leftPaddlePosition);
    positionCommandInterface.registerHandle(rightPaddlePosition);
    registerInterface(&positionCommandInterface);

    // Enable arduino
    std::string initString = arduino.readline(256, "\r");
    arduino.write("H 0\r");
}

// End communication and tear down
PhoenixbotInterface::~PhoenixbotInterface() {
    disable();
    arduino.flush();
    arduino.close();
}

// Disable all powered outputs
void PhoenixbotInterface::disable() {
    arduino.write("H 1\r");
}

// Enable all powered outputs
void PhoenixbotInterface::enable() {
    arduino.write("H 0\r");
}

// Queue up a solenoid command to be sent on next write
void PhoenixbotInterface::solenoid(phoenixbot_msgs::Solenoid msg) {
    for(int i = 0; i < 4; i++) {
        cmdSolenoid[i] = msg.solenoids[i];
    }
}

// Generate a light sensor message from the most recent data
phoenixbot_msgs::Light PhoenixbotInterface::light() {
    phoenixbot_msgs::Light lightMessage;

    for(int i = 0; i < 4; i++) {
        lightMessage.sensors.push_back((float)lightSensors[i]);
    }
    return lightMessage;
}

// Read the state of everything from the robot
void PhoenixbotInterface::read() {
    arduino.flushOutput();
    arduino.flushInput();

    // Read drive encoders
    arduino.write("E -1\r");
    ROS_INFO_STREAM("E -1");

    std::stringstream serialString;
    serialString.str(arduino.readline(256, "\r"));
    ROS_INFO_STREAM(serialString.str());

    for(int i = 0; i < 2; i++) {
        int invert = (i == 1) ? -1 : 1;

        int encoderCounts;
        serialString >> encoderCounts;
        pos[i] = encoderCounts * TICKS_TO_RAD * invert;

        float velocityCounts;
        serialString >> velocityCounts;
        vel[i] = velocityCounts * TICKS_TO_RAD * invert;
    }

    int encoderCounts;
    serialString >> encoderCounts;
    pos[2] = encoderCounts * TICKS_TO_RAD_SIMON;

    float velocityCounts;
    serialString >> velocityCounts;
    vel[2] = velocityCounts * TICKS_TO_RAD_SIMON;

    // TODO Read feedback from other motors
    pos[3] = cmdPos[1];
    pos[4] = cmdPos[2];

    // Read light sensors
    for(int i = 0; i < 4; i++) {
        std::stringstream serialString;
        serialString << "A " << i << "\r";
        arduino.write(serialString.str());
        
        int counts;
        serialString.str(arduino.readline(256, "\r"));
        ROS_INFO_STREAM(serialString.str());
        serialString >> counts;

        lightSensors[i] = counts / 1023.0;
    }
}

// Push commands to the robot
void PhoenixbotInterface::write() {
    std::stringstream serialString;

    // Write left drive motor speed
    serialString << "C 0 S " << (int)(cmdVel[0] / TICKS_TO_RAD * 1000) << "\r";
    arduino.write(serialString.str());
    ROS_INFO_STREAM(serialString.str());
    serialString.str("");

    // Write right drive motor speed
    serialString << "C 1 S " << (int)(-cmdVel[1] / TICKS_TO_RAD * 1000) << "\r";
    arduino.write(serialString.str());
    ROS_INFO_STREAM(serialString.str());
    serialString.str("");

    // Right target arm position
    serialString << "C 2 S " << (int)(cmdPos[0] / TICKS_TO_RAD_SIMON * 1000) << "\r";
    arduino.write(serialString.str());
    ROS_INFO_STREAM(serialString.str());
    serialString.str("");

    // Write paddle positions
    serialString << "M 6 " << (int)(cmdPos[1] / MICROS_TO_RAD) << "\r";
    arduino.write(serialString.str());
    ROS_INFO_STREAM(serialString.str());
    serialString.str("");

    serialString << "M 7 " << (int)(cmdPos[2] / MICROS_TO_RAD) << "\r";
    arduino.write(serialString.str());
    ROS_INFO_STREAM(serialString.str());
    serialString.str("");

    // Write solenoid commands
    for(int i = 0; i < 4; i++) {
        serialString.str("");

        serialString << "S " << solenoidMap[i] << " " << (cmdSolenoid[i] ? 1 : 0) << "\r";
        ROS_INFO_STREAM(serialString.str());
        arduino.write(serialString.str());
    }
}

