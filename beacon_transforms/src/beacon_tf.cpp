#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

//Custom msgs
#include <marvelmind_nav/hedge_pos_ang.h>
#include <marvelmind_nav/hedge_imu_fusion.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


//Function Prototype
void beacon_position(const marvelmind_nav::hedge_pos_ang msg);

void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped msg);

void beaconImuToImu(marvelmind_nav::hedge_imu_fusion msg);

void init_covariances(ros::NodeHandle &nh_private_);

//Global variable
Eigen::Affine3d pose_transform;
Eigen::Affine3d local_pose;
ros::Publisher beacon_pub;
ros::Publisher imu_pub;
sensor_msgs::Imu imu;
geometry_msgs::PoseWithCovarianceStamped pos;

//Main
int main(int argc, char **argv){
	ros::init(argc, argv, "beacon_tf");
	ros::NodeHandle n;
	ros::NodeHandle n_private_("~");
	init_covariances(n_private_);
	beacon_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/beacon_pose",100);
	imu_pub = n.advertise<sensor_msgs::Imu>("/beacon_imu",100);

	{
		ros::Subscriber initialPose_sub = n.subscribe("/initialpose", 100, setInitialPose);
		ros::Subscriber headge_sub = n.subscribe("/hedge_pos_ang", 100, beacon_position);
		ros::Subscriber hedge_imu_sub = n.subscribe("/hedge_imu_fusion", 100, beaconImuToImu);

		ROS_INFO("Running...");
		ros::spin();
	}

	return 0;
}


void beacon_position(const marvelmind_nav::hedge_pos_ang msg){
	Eigen::Affine3d current_pose;

	pos.header.stamp = ros::Time::now();
	pos.header.frame_id = "map";

	// Create affine3d from msg

	local_pose = Eigen::Translation3d(msg.x_m, -msg.y_m, msg.z_m - 0.86);

	// local_pose * transform = global_pose
	current_pose = local_pose * pose_transform;

	// Turn transformed matrix back into message
	tf::poseEigenToMsg(current_pose, pos.pose.pose);
	pos.pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg.angle);

	// Publish the message
	beacon_pub.publish(pos);
	ROS_INFO("Pos has be published");
}

void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped msg){
	Eigen::Affine3d initialPose;
	tf::poseMsgToEigen(msg.pose.pose, initialPose);

	pose_transform = local_pose.inverse() * initialPose;
	ROS_INFO("Postion transform has been set");
}

void beaconImuToImu(marvelmind_nav::hedge_imu_fusion msg){
	imu.header.stamp = ros::Time::now();
	imu.header.frame_id = "beacon";

  imu.orientation.x = msg.qx;
  imu.orientation.y = msg.qy;
  imu.orientation.z = msg.qz;
  imu.orientation.w = msg.qw;

  imu.angular_velocity.x = msg.vx;
  imu.angular_velocity.y = msg.vy;
  imu.angular_velocity.z = msg.vz;

  imu.linear_acceleration.x = msg.ax;
  imu.linear_acceleration.y = msg.ay;
  imu.linear_acceleration.z = msg.az;

  imu_pub.publish(imu);

}

void init_covariances(ros::NodeHandle &n_private_){
    // Create the vectors to store the covariance matrix arrays
    std::vector<double> orientation_covar;
    std::vector<double> ang_vel_covar;
    std::vector<double> linear_accel_covar;
    std::vector<double> pose_covar;

    // Grab the parameters and populate the vectors
    n_private_.getParam("imu_orientation_covariance", orientation_covar);
    n_private_.getParam("imu_angular_velocity_covariance", ang_vel_covar);
    n_private_.getParam("imu_linear_acceleration_covariance", linear_accel_covar);
    n_private_.getParam("pose_covariance", pose_covar);

    // Iterate through each vector and populate the respective message fields
    for (int i = 0; i < orientation_covar.size(); i++)
      imu.orientation_covariance[i] = orientation_covar.at(i);

    for (int i = 0; i < ang_vel_covar.size(); i++)
      imu.angular_velocity_covariance[i] = ang_vel_covar.at(i);

    for (int i = 0; i < linear_accel_covar.size(); i++)
      imu.linear_acceleration_covariance[i] = linear_accel_covar.at(i);

    for (int i = 0; i < pose_covar.size(); i++)
      pos.pose.covariance[i] = pose_covar.at(i);
  }
