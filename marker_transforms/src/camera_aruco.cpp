/*
 * Camera Aruco
 * Get Aruco Marker positions from the camera
 *
 * Ryan Owens
 *
 */

#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "aruco.h"

tf2::Quaternion *convertToQuaternion(double rotx, double roty, double rotz);

int main(int argc, char **argv) {
        ros::init(argc, argv, "marker_transforms");
        ros::NodeHandle n;
        ros::Rate loop_rate(10);
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        std::vector<geometry_msgs::TransformStamped> transforms;
        Aruco *arucoDetector = new Aruco(0);
        if (!arucoDetector->openCamera()) {
                std::cerr << "Could not open the camera" << std::endl;
                return -1;
        }
        while (ros::ok()) {
                std::vector<int> markers = arucoDetector->arucoMarkersInView();
                for (auto &marker : markers) {
                        std::vector<double> directPose = arucoDetector->getPose(marker);
                        if (directPose.size() >= 6) {
                                transformStamped.header.stamp = ros::Time::now();
                                transformStamped.header.frame_id = "world";
                                transformStamped.child_frame_id = std::to_string(marker);
                                transformStamped.transform.translation.x = directPose[0];
                                transformStamped.transform.translation.y = directPose[1];
                                transformStamped.transform.translation.z = directPose[2];
                                tf2::Quaternion *q =
                                        convertToQuaternion(directPose[3], directPose[4], directPose[5]);
                                transformStamped.transform.rotation.x = q->x();
                                transformStamped.transform.rotation.y = q->y();
                                transformStamped.transform.rotation.z = q->z();
                                transformStamped.transform.rotation.w = q->w();
                                transforms.push_back(transformStamped);
                                delete (q);
                                q = nullptr;
                        }
                }
                br.sendTransform(transforms);
                transforms.clear();
                ros::spinOnce();
                loop_rate.sleep();
        }
        delete (arucoDetector);
        arucoDetector = nullptr;

        return 0;
}

tf2::Quaternion *convertToQuaternion(double rotx, double roty, double rotz) {
        float angle =
                (float)(sqrt(rotx * rotx + roty * roty + rotz + rotz) * (180 / M_PI));
        tf2::Vector3 axis(-rotx, roty, -rotz);
        return new tf2::Quaternion(axis, angle);
}
