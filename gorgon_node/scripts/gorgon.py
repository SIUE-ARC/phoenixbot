#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

def sendToGorgon(string):
	rospy.loginfo("Sending " + str(string) + " to gorgon")

def readFromGorgon():
#	rospy.loginfo("Reading from gorgon")
	return "1024" 
	
def getResolution(i):
	resolution = rospy.get_param("enc" + str(i) + "/resolution")
	return resolution
	
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

def readEncoder():
	encoder = range(0,8)
	# Ask the gorgon what the analog values are
	for i in range(0,8):
		#sendToGorgon("E " + str(i))
		encoder[i] = readFromGorgon()
		encoder = map(int, encoder)
		rospy.loginfo(encoder[i])
	#convert to radians (float)
	for i in range(0,8):
		res = getResolution(i)
		if(res == 0):
			res = 1
			encoder[i] = (encoder[i] * 0)
		encoder[i] = encoder[i]/res
		rospy.loginfo(encoder[i])
	joint.position = encoder






#This is an example of how you publish multiple data under 1 message
#	joint = JointState()
#	joint.name = ["Encoder 1"]  	< Must be a list or array
#	joint.position = [2]	    	< Must be a list or array
#	joint.velocity = [8.2]      	< Must be a list or array
#	joint.effort = [3.4]        	< Must be a list or array
#	joint_state_pub.publish(joint)
#Note: Important to check the name/type of data in the message 
	

if __name__ == "__main__":
    # Entry point
    rospy.init_node('gorgon_node')

    joint = JointState() #The message

    analog_input_pub = rospy.Publisher('analog_input',  Float32MultiArray, queue_size=1)
    encoder_input_pub = rospy.Publisher('encoder_input',  Int32MultiArray, queue_size=1)
    joint_state_pub = rospy.Publisher('joint_state', JointState, queue_size=1)   

    rate = rospy.Rate(10) # 10hz = 1/10 sec
    while not rospy.is_shutdown():
	readAnalog()
	readEncoder()
	joint_state_pub.publish(joint)
        rate.sleep()

