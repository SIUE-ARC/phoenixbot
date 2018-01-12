#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def sendToGorgon(string):
	rospy.loginfo("Sending " + str(string) + " to gorgon")

def readFromGorgon():
	rospy.loginfo("Reading from gorgon")
	return "1024" 

def readAnalog():
	values = range(0,5)
	# Ask the gorgon what the analog values are
	for i in range(0,5):
		sendToGorgon("A " + str(i))
		values[i] = readFromGorgon()
	values = map(float, values)

	# Publish those values to nodes that are listening
	message = Float32MultiArray()
	message.data = values
	analog_input_pub.publish(message)

if __name__ == "__main__":
    # Entry point
    rospy.init_node('gorgon_node')

    analog_input_pub = rospy.Publisher('analog_input',  Float32MultiArray, queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	readAnalog()
        rate.sleep()

