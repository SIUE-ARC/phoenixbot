#!/usr/bin/env python
import rospy

import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from phoenixbot_msgs.msg import Light
from phoenixbot_msgs.msg import Solenoid
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from std_msgs.msg import Float64

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

def drive_to(waypoint):
    global active_goal_status
    nav_target.publish(waypoint)
    while active_goal_status != 3:
        pass
    active_goal_status = 0

def approach_simon():
    blocking_drive(-0.6, 0, 1)

def raise_simon():
    msg = Float64()
    msg.data = 0
    simon_arm_cmd.publish(msg)
    rospy.sleep(1)

def lower_simon():
    msg = Float64()
    msg.data = 1.6
    simon_arm_cmd.publish(msg)
    rospy.sleep(1)

def do_simon():
    lower_simon()
    while simon_light != -1:
        s = simon_light
        solenoid_msg = Solenoid()
        solenoid_msg.solenoids = [False] * 4
        if s >= 0:
            solenoid_msg.solenoids[s] = True
            solenoid_pub.publish(solenoid_msg)
            rospy.sleep(1)
            solenoid_msg.solenoids[s] = False
            solenoid_pub.publish(solenoid_msg)
            rospy.sleep(1)
    raise_simon()

def pull_rope():
    msg = Float64()
    msg.data = 0.75
    rope_cmd.publish(msg)

def stop_rope():
    msg = Float64()
    msg.data = 0.0
    rope_cmd.publish(msg)

def release_rope():
    msg = Float64()
    msg.data = -0.75
    rope_cmd.publish(msg)

def approach_pulley():
    pass

def do_pulley():
    pull_rope()
    blocking_drive(0.5, 0, 1)
    rospy.sleep(3)
    stop_rope()
    rospy.sleep(1)
    release_rope()
    blocking_drive(-0.5, 0, 1)
    rospy.sleep(4)
    stop_rope()

def drive_down_ramp():
    # Drive down the ramp
    print("Driving to ramp...");
    while not ramp:
        drive(0.75, 0.0)
    print("Driving down ramp...");
    while ramp:
        drive(0.75, 0.0)
    print("Reached bottom of ramp");
    drive(0.0, 0.0)


def competition():
    pose_pub.publish(get_pose("initial_pose"))
    wait_for_start()

    drive_down_ramp()

    # Update the pose
    print("Updating pose...")
    pose_pub.publish(get_pose("bottom_of_ramp"))
    clear_costmap()
    rospy.sleep(1)

    # Drive clear of the ramp
    print("Clearing ramp")
    blocking_drive(1.0, 0.0, 0.75)

    print("Driving to simon")
    drive_to(get_waypoint("simon_approach"))

    print("Approaching simon")
    approach_simon()

    print("Doing simon")
    do_simon()

    print("Driving to pulley")
    drive_to(get_waypoint("pulley_approach"))

    print("Approaching pulley")
    approach_pulley()

    print("Doing pulley")
    do_pulley()
        
# Initialization
rospy.init_node('phoenixbot_behavior_coordinator')

imu_sub = rospy.Subscriber("imu_data", Imu, imu_callback)
nav_result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, nav_callback)
simon_lights = rospy.Subscriber("light_sensors", Light, light_callback)

pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=3)
vel_pub = rospy.Publisher('move_base_controller/cmd_vel', Twist, queue_size=3)
nav_target = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
solenoid_pub = rospy.Publisher("solenoid_commands", Solenoid, queue_size=1)
rope_cmd = rospy.Publisher('/rope_controller/command', Float64, queue_size=3)
simon_arm_cmd = rospy.Publisher('/simon_arm_controller/command', Float64, queue_size=3)

# rospy.wait_for_service('/move_base/clear_costmaps')
clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

# Start
competition()

# Tests
# do_simon()
# do_pulley()


