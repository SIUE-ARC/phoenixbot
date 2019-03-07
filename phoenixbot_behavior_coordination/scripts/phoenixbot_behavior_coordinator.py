#!/usr/bin/env python
import rospy

import tf2_ros

import numpy
import quaternion

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from phoenixbot_msgs.msg import Light
from phoenixbot_msgs.msg import Solenoid
from phoenixbot_msgs.msg import Markers
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from std_msgs.msg import Float64
import math

import tf.transformations

# TODO
def approach_simon():
    blocking_drive(-0.6, 0, 1)

ramp = False
def imu_callback(data):
    global ramp

    orientation = numpy.quaternion(
        data.orientation.w,
        data.orientation.x,
        data.orientation.y,
        data.orientation.z 
    ) 
    z_hat = numpy.array([0,0,1]);

    robot_up = quaternion.rotate_vectors(orientation, z_hat)
    deviation = 1-z_hat.dot(robot_up);

    ramp = deviation > 0.025;

active_goal_status = 0
def nav_callback(msg):
    global active_goal_status
    active_goal_status = msg.status.status

simon_light = -2
def light_callback(msg):
    global simon_light
    simon_light = -2

    max_intensity = 0
    average_intensity = 0
    for i, value in enumerate(msg.sensors):
        average_intensity += value;
        if value > max_intensity:
            max_intensity = value
            if value > 0.5:
                simon_light = i
    average_intensity /= 4

    if average_intensity > 0.8:
        simon_light = -1

markers = {}
def marker_callback(msg):

    for id, pose in zip(msg.ids, msg.poses):
        p = PoseStamped()
        p.header = msg.header
        p.pose = pose
        markers[id] = p

def wait_for_start():
    pass

def get_waypoint(waypoint_name):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = rospy.get_param('~{}/position/x'.format(waypoint_name))
    pose.pose.position.y = rospy.get_param('~{}/position/y'.format(waypoint_name))
    pose.pose.position.z = rospy.get_param('~{}/position/z'.format(waypoint_name))

    if(rospy.has_param('~{}/orientation/r'.format(waypoint_name))):
        r = rospy.get_param('~{}/orientation/r'.format(waypoint_name))
        p = rospy.get_param('~{}/orientation/p'.format(waypoint_name))
        y = rospy.get_param('~{}/orientation/y'.format(waypoint_name))

        quaternion = tf.transformations.quaternion_from_euler(r, p, y)

        pose.pose.orientation.w = quaternion[0]
        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]
    else:
        pose.pose.orientation.x = rospy.get_param('~{}/orientation/x'.format(waypoint_name))
        pose.pose.orientation.y = rospy.get_param('~{}/orientation/y'.format(waypoint_name))
        pose.pose.orientation.z = rospy.get_param('~{}/orientation/z'.format(waypoint_name))
        pose.pose.orientation.w = rospy.get_param('~{}/orientation/w'.format(waypoint_name))
    return pose

def get_pose(pose_name):
    pose_covarience = PoseWithCovarianceStamped()
    pose_stamped = get_waypoint(pose_name)

    pose_covarience.header = pose_stamped.header
    pose_covarience.pose.pose = pose_stamped.pose
    return pose_covarience

def blocking_drive(drive_speed, turn_speed, drive_time):
    for n in range(0, int(drive_time * 10)):
        drive(drive_speed, turn_speed)
        rospy.sleep(0.1)
    drive(0.0, 0.0)

def drive(drive_speed, turn_speed):
    cmd_msg = Twist()
    cmd_msg.linear.x = drive_speed
    cmd_msg.angular.z = turn_speed
    vel_pub.publish(cmd_msg)

def drive_distance(distance, time=None):
    if time is None: 
        time = abs(distance) / 0.5
    blocking_drive(distance / time, 0, time)

def drive_to(waypoint):
    global active_goal_status
    nav_target.publish(waypoint)
    r = rospy.Rate(10)

    while active_goal_status != 3:
        if active_goal_status == 4:
            active_goal_status = 0
            raise Exception("Goal aborted")
        r.sleep()

    active_goal_status = 0

def raise_simon():
    msg = Float64()
    msg.data = 0
    simon_arm_cmd.publish(msg)

def lower_simon():
    msg = Float64()
    msg.data = -1.6
    simon_arm_cmd.publish(msg)

def do_simon():
    start_time = rospy.Time.now();
    servo_states = {
        0: (-1.0,  0.0),
        1: ( 0.0,  1.0),
        2: ( 0.0, -1.0),
        3: ( 1.0,  0.0)
    }
    while simon_light != -1 and rospy.Time.now() - start_time < rospy.Duration(10):
        s = simon_light
        if s >= 0:
            msg = Float64()

            msg.data = servo_states[s][0]
            left_paddle_pub.publish(msg)

            msg.data = servo_states[s][1]
            right_paddle_pub.publish(msg)

            rospy.sleep(1.0)

            msg.data = 0.0
            right_paddle_pub.publish(msg)
            left_paddle_pub.publish(msg)

            rospy.sleep(1.0)

    if simon_light != -1:
        raise Exception("Simon timed out")

def cross_ramp():
    print("Driving to ramp...");
    while not ramp:
       drive(0.5, 0.0)

    print("Driving through ramp...");
    while ramp:
       drive(0.5, 0.0)

    print("Reached end of ramp");
    drive(0.0, 0.0)

def competition():
    print("Crossing stage")
    while True:
        try:
            drive_to(get_waypoint("opposite_ramp_top"))
            break
        except:
            print("Attempting recovery")
            drive_distance(-0.2);

    print("Clearing ramp")
    cross_ramp()
    drive_distance(0.5)

    print("Driving to simon")
    drive_to(get_waypoint("simon_approach"))

    print("Approaching simon")
    drive_distance(-0.6)

    print("Completing simon")
    lower_simon()
    try:
        do_simon()
    except:
        pass
    raise_simon()
    drive_distance(0.6)

    print("Driving to ramp")
    drive_to(get_waypoint("opposite_ramp_bottom"))
    cross_ramp()
    drive_distance(0.5)
        
# Initialization
rospy.init_node('phoenixbot_behavior_coordinator')

imu_sub = rospy.Subscriber("imu_data", Imu, imu_callback)

nav_result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, nav_callback)
simon_lights = rospy.Subscriber("light_sensors", Light, light_callback)
marker_sub = rospy.Subscriber('/markers', Markers, marker_callback)

pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=3)

vel_pub = rospy.Publisher('/direct/cmd_vel', Twist, queue_size=3)
nav_target = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)

left_paddle_pub = rospy.Publisher("/left_paddle_controller/command", Float64, queue_size=1)
right_paddle_pub = rospy.Publisher("/right_paddle_controller/command", Float64, queue_size=1)
simon_arm_cmd = rospy.Publisher('/simon_arm_controller/command', Float64, queue_size=3)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# rospy.wait_for_service('/move_base/clear_costmaps')
clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
rospy.sleep(5)

# Start
pose_pub.publish(get_pose("initial_pose"))
wait_for_start()
competition()

