#include <controller_manager/controller_manager.h>

#include "phoenixbot_driver/PhoenixbotInterface.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "phoenixbot_driver");
    ros::NodeHandle nh;

    std::string port;
    int baud;

    if (!ros::param::get("~port", port)) {
      port = "/dev/ttyACM0";
      ROS_WARN_STREAM("No port specified, using default: " << port);
    }

    if (!ros::param::get("~baudrate", baud)) {
      baud = 1000000;
      ROS_WARN_STREAM("No baudrate specified, using default: " << baud);
    }

    PhoenixbotInterface interface(port, baud, 1000);
    controller_manager::ControllerManager cm(&interface);

    // Spin off a thread to handle ROS interactions so main thread remains realtime
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Realtime control starts here
    ros::Time then = ros::Time::now();
    ros::Rate rate(10.0);

    ros::Publisher sensorTopic = nh.advertise<phoenixbot_msgs::Light>("light_sensors", 1);
    ros::Subscriber solenoidTopic = nh.subscribe("solenoid_commands", 1, &PhoenixbotInterface::solenoid, &interface);

    while (ros::ok()) {
        const ros::Time now = ros::Time::now();

        interface.read();
        sensorTopic.publish(interface.light());
        cm.update(now, now - then);
        interface.write();

        then = now;
        rate.sleep();
    }
}

