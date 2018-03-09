#!/usr/bin/env python
import rospy
import numpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from phoenixbot_msgs.msg import Light
from phoenixbot_msgs.msg import Solenoid
from std_msgs.msg import Float64

def light_callback(msg):
    print(msg)

def raise_simon():
    msg = Float64()
    msg.data = 0
    simon_arm_cmd.publish(msg)

def lower_simon():
    msg = Float64()
    msg.data = 1.6
    simon_arm_cmd.publish(msg)

def joy_callback(msg):
    solenoid_msg = Solenoid()
    solenoid_msg.solenoids = msg.buttons[0:4]
    solenoid_pub.publish(solenoid_msg)

    twist = Twist()
    twist.angular.z = msg.axes[0] * 3
    twist.linear.x = msg.axes[1] * 3
    vel_pub.publish(twist)

    if 1 - ((msg.axes[2] + 1) / 2.0) > 0.1:
        rope_speed = msg.axes[2]
    elif 1 - ((msg.axes[5] + 1) / 2.0) > 0.1:
        rope_speed = -msg.axes[5]
    else:
        rope_speed = 0

    cmd = Float64()
    cmd.data = rope_speed
    rope_cmd.publish(cmd)

    if msg.buttons[5]:
        lower_simon()
    elif msg.buttons[4]:
        raise_simon()


joy_sub = rospy.Subscriber("joy", Joy, joy_callback)
simon_lights = rospy.Subscriber("light_sensors", Light, light_callback)

vel_pub = rospy.Publisher('move_base_controller/cmd_vel', Twist, queue_size=3)
solenoid_pub = rospy.Publisher("solenoid_commands", Solenoid, queue_size=1)

rope_cmd = rospy.Publisher('/rope_controller/command', Float64, queue_size=3)
simon_arm_cmd = rospy.Publisher('/simon_arm_controller/command', Float64, queue_size=3)

rospy.init_node("phoenixbot_teleop")
rospy.spin()

