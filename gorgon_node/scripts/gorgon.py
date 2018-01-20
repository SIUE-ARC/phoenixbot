#!/usr/bin/env python
import rospy
from gorgon_node.msg import custom

def sendToGorgon(string):
	rospy.loginfo("Tx: " + str(string))

def setSpeed(message):
	value = int(((message.speed+1)/2.0)*255)
	sendToGorgon("M " + str(message.MotorID) + " " + str(value))


if __name__ == "__main__":
    # Entry point
    rospy.init_node('gorgon_node')

    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("Motor_speed",custom, setSpeed)
    while not rospy.is_shutdown():
	rate.sleep()
rospy.spin()

#Create message to controll individual motor

