#include <controller_manager/controller_manager.h>

#include <serial/serial.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/Imu.h>

#include <eigen_conversions/eigen_msg.h>

#define RAD 0.0174533

#define PACKET_START 0x55
#define ACCEL_PACKET 0x51
#define ANG_VEL_PACKET 0x52
#define ORIENTATION_PACKET 0x53

#define CALIBRATED 0x1
#define ACCEL_READY 0x2
#define ANG_READY 0x04
#define ORIENTATION_READY 0x08

sensor_msgs::Imu prepareMessage(Eigen::Vector3d linear, Eigen::Vector3d angular, Eigen::Vector3d rpy, Eigen::Vector3d calibration) {
  sensor_msgs::Imu imu;
  imu.header.stamp = ros::Time::now();
  ros::param::param<std::string>("frame_id", imu.header.frame_id, "imu_link");

  Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) * 
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());

  tf::quaternionEigenToMsg(orientation, imu.orientation);
  imu.orientation_covariance = {0.01, 0.0, 0.0,  
                                0.0, 0.01, 0.0,  
                                0.0, 0.0, 0.01};

  tf::vectorEigenToMsg(angular, imu.angular_velocity);
  imu.angular_velocity_covariance = {0.2, 0.0, 0.0,  
                                     0.0, 0.2, 0.0,  
                                     0.0, 0.0, 0.2};
  
  tf::vectorEigenToMsg(linear - calibration, imu.linear_acceleration);
  imu.linear_acceleration_covariance = {0.1, 0.0, 0.0,  
                                        0.0, 0.1, 0.0,  
                                        0.0, 0.0, 0.1};
  return imu;
}

uint8_t parsePacket(uint8_t* buffer, Eigen::Vector3d& a, Eigen::Vector3d& omega, Eigen::Vector3d& rpy) {
  uint8_t& header   = buffer[0];
  uint8_t& type     = buffer[1];
  int16_t x        = buffer[3] << 8 | buffer[2];
  int16_t y        = buffer[5] << 8 | buffer[4];
  int16_t z        = buffer[7] << 8 | buffer[6];
  uint16_t t        = buffer[9] << 8 | buffer[8];
  uint8_t& checksum = buffer[10];

  Eigen::Vector3d v(x, y, z);

  // Calculate checksum
  uint8_t sum = 0;
  for (int b = 0; b < 10; b++) {
    sum += buffer[b];
  }

  if (header != PACKET_START) {
    ROS_WARN("Read skew occured");
    return 0;
  }

  if (sum != checksum) {
    ROS_WARN("Checksum validation failed. %d did not match expected %d", sum, checksum);
    return 0;
  }

  if (type == ACCEL_PACKET) {
    a = (v / 32768) * 16 * 9.81;
    return ACCEL_READY;
  }

  else if (type == ANG_VEL_PACKET) {
    omega = (v / 32768) * 2000 * RAD;
    return ANG_READY;
  }

  else if (type == ORIENTATION_PACKET) {
    rpy = (v / 32768) * 180 * RAD;
    return ORIENTATION_READY;
  }

  ROS_WARN("Unknown packet recieved");
  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wit_imu");
  ros::NodeHandle nh;

  std::string port;
  int baud;
  int timeout;

  if (!ros::param::get("~port", port)) {
    port = "/dev/ttyUSB0";
    ROS_WARN_STREAM("No port specified, using default: " << port);
  }

  if (!ros::param::get("~baudrate", baud)) {
    baud = 115200;
    ROS_WARN_STREAM("No baudrate specified, using default: " << baud);
  }

  if (!ros::param::get("~timeout", baud)) {
    timeout = 250;
    ROS_WARN_STREAM("No timeout specified, using default: " << timeout);
  }

  serial::Serial imu(port, baud, serial::Timeout::simpleTimeout(timeout));

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);

  uint8_t buffer[11];
  int b = 0;

  uint8_t flags = 0;
  unsigned int calibration_samples = 0;
  Eigen::Vector3d accel_calibration;
  Eigen::Vector3d a;
  Eigen::Vector3d omega;
  Eigen::Vector3d rpy;

  ROS_INFO("Calibrating IMU");
  imu.write("\xFF\xAA\x52");
  imu.write("\xFF\xAA\x67");
  imu.write("\xFF\xAA\x65");

  while (ros::ok()) {
    // Read acceleration data into buffer
    imu.read(buffer + b, 1);
    if (buffer[0] == PACKET_START)
      b++;

    if (b == 11) {
      b = 0;

      flags |= parsePacket(buffer, a, omega, rpy);

      if (!(flags & CALIBRATED)) {
        if (flags & ACCEL_READY) {
          accel_calibration += a;
          calibration_samples++;
          flags &= ~ACCEL_READY;
        }

        if (calibration_samples >= 100) {
          Eigen::Vector3d gravity(0,0,9.81);
          accel_calibration /= calibration_samples;
          accel_calibration = gravity - accel_calibration;
          flags |= CALIBRATED;
          ROS_INFO("Calibration complete");
        }
      }

      if (flags == (CALIBRATED | ACCEL_READY | ANG_READY | ORIENTATION_READY)) {
        imu_pub.publish(prepareMessage(a, omega, rpy, accel_calibration));
        flags &= ~(ACCEL_READY | ANG_READY | ORIENTATION_READY);
      }
    }
  }
}

