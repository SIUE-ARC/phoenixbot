#include "phoenixbot_driver/PhoenixbotInterface.h"

#include <sstream>

PhoenixbotInterface::PhoenixbotInterface(std::string port, int baud, int timeout)
    : arduino(port, baud, serial::Timeout::simpleTimeout(timeout))
{
    // Setup ros_control interfaces
    // State Handles
    hardware_interface::JointStateHandle leftWheelStateHandle          ("left_wheel_joint",     pos + 0, vel + 0, eff + 0);
    hardware_interface::JointStateHandle righWheeltStateHandle         ("right_wheel_joint",    pos + 1, vel + 1, eff + 1);
    hardware_interface::JointStateHandle leftRopeStateHandle      ("left_rope_joint",   pos + 2, vel + 2, eff + 2);
    hardware_interface::JointStateHandle rightRopeStateHandle     ("right_rope_joint", pos + 3, vel + 3, eff + 3);
    hardware_interface::JointStateHandle simonPusherStateHandle   ("simon_pusher_joint", pos + 4, vel + 4, eff + 4);

    // Register state handles
    stateInterface.registerHandle(leftWheelStateHandle);
    stateInterface.registerHandle(rightWheelStateHandle);
    stateInterface.registerHandle(leftRopeStateHandle);
    stateInterface.registerHandle(rightRopeStateHandle);
    stateInterface.registerHandle(simonPusherStateHandle);

    // Velocity handles
    hardware_interface::JointHandle leftVelocityHandle  (leftWheelStateHandle,  cmdVel + 0);
    hardware_interface::JointHandle rightVelocityHandle (rightWheelStateHandle, cmdVel + 1);

    // Register velocity handles
    velocityCommandInterface.registerHandle(leftVelocityHandle);
    velocityCommandInterface.registerHandle(rightVelocityHandle);

	// Effort handles
	hardware_interface::JointHandle leftRopeEffortHandle     (leftRopeStateHandle, cmdEff + 0);
	hardware_interface::JointHandle rightRopeEffortHandle    (rightRopeStateHandle, cmdEff + 1);
	hardware_interface::JointHandle simonPusherEffortHandle  (simonPusherStateHandle, cmdEff + 2);

	// Register effort handles
	effortCommandInterface.registerHandle(leftRopeStateHandle);
	effortCommandInterface.registerHandle(rightRopeStateHandle);
	effortCommandInterface.registerHandle(simonPusherStateHandle);


    registerInterface(&stateInterface);
    registerInterface(&velocityCommandInterface);
    registerInterface(&effortCommandInterface);

}

// TODO End communication and tear down
PhoenixbotInterface::~PhoenixbotInterface() {
}

// Read the state of everything from the robot
void PhoenixbotInterface::read() {
	arduino.write("E -1\r");
	String record = arduino.readline();
	
    unsigned long tick;
	int res[2];
	for(int j = 0; j < 2; j++)
	{
		res[j] = ros::param::get("enc" + j + "/resolution");
	}

	Stringstream ss(record);
	int i = 0;
	while(!ss.eof()) {
		ss >> tick;
		pos[i] = tick / res[i];
		i++;
	}
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

