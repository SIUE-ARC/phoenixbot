#!/usr/bin/env python
import rospy

import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import tf.transformations

z_hat = numpy.array([0,0,1]);

ramp = False
max_deviation = 1 - 0.9840;

def imu_callback(data):
    global max_deviation
    global ramp

    rot_matrix = tf.transformations.quaternion_matrix(numpy.array([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]));

    robot_up = numpy.array([data.orientation.x, data.orientation.y, data.orientation.z])
    robot_up = robot_up / numpy.linalg.norm(robot_up);
    deviation = 1-z_hat.dot(robot_up);

    max_deviation = max(deviation, max_deviation);

    if deviation > (max_deviation * 0.05):
        ramp = True;
    else:
        ramp = False;

    print(deviation, ramp)

def wait_for_start():
    pass

def drive(drive_speed, turn_speed):
    cmd_msg = Twist()
    cmd_msg.linear.x = drive_speed
    cmd_msg.angular.z = turn_speed
    vel_pub.publish(cmd_msg)

def get_ramp_pose():
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = -4.791
    pose.pose.pose.position.y =  4.840
    pose.pose.pose.position.z =  0.000
    pose.pose.pose.orientation.x =  0.000
    pose.pose.pose.orientation.y =  0.000
    pose.pose.pose.orientation.z =  0.928
    pose.pose.pose.orientation.w =  0.372
    return pose

def get_off_pose():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = -5.5
    pose.pose.position.y =  5.5
    pose.pose.position.z =  0.000
    pose.pose.orientation.x =  0.000
    pose.pose.orientation.y =  0.000
    pose.pose.orientation.z =  0.928
    pose.pose.orientation.w =  0.372
    return pose

def drive_down_ramp():
    print("Driving to ramp...");
    while not ramp:
        drive(0.5, 0.0)
    print("Driving down ramp...");
    while ramp:
        drive(0.5, 0.0)
    print("Reached bottom of ramp");
    drive(0.0, 0.0)

# Initialization
pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=3)
vel_pub = rospy.Publisher('move_base_controller/cmd_vel', Twist, queue_size=3)
imu_sub = rospy.Subscriber("imu_data", Imu, imu_callback)
nav_target = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=3)

rospy.init_node('phoenixbot_behavior_coordinator')

# Start
wait_for_start()

drive_down_ramp()
pose_pub.publish(get_ramp_pose())

drive(0.5, 0.0)
rospy.sleep(3)
drive(0.0, 0.0)

nav_target.publish(get_off_pose())

rospy.spin()

