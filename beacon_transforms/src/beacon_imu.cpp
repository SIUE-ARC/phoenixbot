#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

//Custom message
#include <marvelmind_nav/hedge_imu_fusion.h>

void beaconImuToImu(marvelmind_nav::hedge_imu_fusion msg);

ros::Publisher imu_pub;

int main(int argc, char **argv){
  ros::init(argc, argv, "imu_converter");
	ros::NodeHandle n;

  imu_pub = n.advertise<sensor_msgs::Imu>("/imu",100);
  {
  ros::Subscriber hedge_imu_sub = n.subscribe("/hedge_imu_fusion", 100, beaconImuToImu);
  ros::spin();
  }
  return 0;
}

void beaconImuToImu(marvelmind_nav::hedge_imu_fusion msg){
  sensor_msgs::Imu data;

  data.header.stamp = ros::Time::now();

  data.orientation.x = msg.qx;
  data.orientation.y = msg.qy;
  data.orientation.z = msg.qz;
  data.orientation.w = msg.qw;

  data.angular_velocity.x = msg.vx;
  data.angular_velocity.y = msg.vy;
  data.angular_velocity.z = msg.vz;

  data.linear_acceleration.x = msg.ax;
  data.linear_acceleration.y = msg.ay;
  data.linear_acceleration.z = msg.az;

  imu_pub.publish(data);

}
