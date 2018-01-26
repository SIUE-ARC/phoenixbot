#include <controller_manager/controller_manager.h>

#include "phoenixbot_driver/PhoenixbotInterface.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "phoenixbot_driver");

    // TODO Move to param server
    PhoenixbotInterface interface("dev/ttyUSB0", 19200, 250);
    controller_manager::ControllerManager cm(&interface);

    // Spin off a thread to handle ROS interactions so main thread remains realtime
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Realtime control starts here
    ros::Time then = ros::Time::now();
    ros::Rate rate(50.0);

    while (ros::ok()) {
        const ros::Time now = ros::Time::now();

        interface.read();
        cm.update(now, now - then);
        interface.write();

        then = now;
        rate.sleep();
    }
}

