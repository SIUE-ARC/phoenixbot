#!/usr/bin/env python  

import rospy

import tf_conversions
import tf2_ros
import geometry_msgs.msg
from aruco_msgs.msg import MarkerArray


def marker_pose(msg):
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	t.header.frame_id = "camera"

	for i in range(len(msg.markers)):
		t.header.stamp = msg.markers[i].header.stamp
		t.child_frame_id = str(msg.markers[i].id)+"_marker_frame"
		t.transform.translation.x = msg.markers[i].pose.pose.position.x
		t.transform.translation.y = msg.markers[i].pose.pose.position.y
		t.transform.translation.z = msg.markers[i].pose.pose.position.z

		t.transform.rotation.x = msg.markers[i].pose.pose.orientation.x
		t.transform.rotation.y = msg.markers[i].pose.pose.orientation.y
		t.transform.rotation.z = msg.markers[i].pose.pose.orientation.z
		t.transform.rotation.w = msg.markers[i].pose.pose.orientation.w
		
		br.sendTransform(t)	



if __name__ == '__main__':
	rospy.init_node('tf2_markers_broadcaster')
	rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, marker_pose)

	rospy.spin()
