#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
rospy.init_node("robot_node")

get_right_motor_param = rospy.get_param("~rightmotorport")
get_left_motor_param = rospy.get_param("~leftmotorport")

print("Right motor param: {}".format(get_right_motor_param))
print("Left motor param: {}".format(get_left_motor_param))


def callback(data):
	print("Linear: {}".format(data.linear))
	print("Angular: {}".format(data.angular))
	drivespeed = data.linear.x
	turnspeed = data.angular.z
	left_motor_speed = turnspeed + drivespeed
	right_motor_speed = turnspeed - drivespeed
	print("Left motor speed: {}".format(left_motor_speed))
	print("Right motor speed: {}".format(right_motor_speed))
	pub = rospy.Publisher("motor_speed", String, queue_size=1)
	message = "{},{}".format(left_motor_speed,right_motor_speed)
	pub.publish(message)


rospy.Subscriber("cmd_vel", Twist, callback)



rospy.spin()

	
