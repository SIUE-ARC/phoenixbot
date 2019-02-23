#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

//Custom msgs
#include <marvelmind_nav/hedge_pos_a.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>


//Function Prototype
void beacon_position(const marvelmind_nav::hedge_pos_a msg);

void setInitialPose(const geometry_msgs::PoseStamped msg);


//Global variable
Eigen::Affine3d pose_transform;
Eigen::Affine3d local_pose;
ros::Publisher beacon_pub;


//Main
int main(int argc, char **argv){
	ros::init(argc, argv, "beacon_tf");
	ros::NodeHandle n;
	beacon_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/current_pose",100);


	{
		ros::Subscriber initialPose_sub = n.subscribe("/initialpose", 100, setInitialPose);
		ros::Subscriber headge_sub = n.subscribe("/hedge_pos_a", 100, beacon_position);
		
		ROS_INFO("Running...");
		ros::spin();
	}
	
	return 0;
}


void beacon_position(const marvelmind_nav::hedge_pos_a msg){
	geometry_msgs::PoseWithCovarianceStamped pos;
	Eigen::Affine3d current_pose;

	// Create affine3d from msg

	local_pose = Eigen::Translation3d(msg.x_m, msg.y_m, msg.z_m);	

	// local_pose * transform = global_pose
	current_pose = local_pose * pose_transform;
	
	// Turn transformed matrix back into message
	tf::poseEigenToMsg(current_pose, pos.pose.pose);	

	// Publish the message
	beacon_pub.publish(pos);
	ROS_INFO("Pos has be published");
}


void setInitialPose(const geometry_msgs::PoseStamped msg){
	Eigen::Affine3d initialPose;
	tf::poseMsgToEigen(msg.pose, initialPose);

	pose_transform = local_pose.inverse() * initialPose;
	ROS_INFO("Postion transform has been set");
}

