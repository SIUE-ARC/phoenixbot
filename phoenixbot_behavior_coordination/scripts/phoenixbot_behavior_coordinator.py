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

simon_light = 0
def light_callback(msg):
    global simon_light

    max_intensity = 0
    for i, value in enumerate(msg.sensors):
        if value > max_intensity:
            max_intensity = value
            simon_light = i

    max_difference = 0
    for i, value in enumerate(msg.sensors):
        if max_intensity - value > max_difference:
            max_difference = max_intensity - value

    if max_difference < 50:
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

def drive(drive_speed, turn_speed):
    cmd_msg = Twist()
    cmd_msg.linear.x = drive_speed
    cmd_msg.angular.z = turn_speed
    vel_pub.publish(cmd_msg)

def drive_to(waypoint):
    nav_target.publish(waypoint)
    while active_goal_status != 3:
        pass

def approach_simon():
    pass

def do_simon():
    solenoid_cmd = Solenoid()
    while simon_light != -1:
        solenoid_cmd.solenoids = [False] * 4
        solenoid_cmd.solenoids[simon_light] = True
        solenoid_pub.publish(solenoid_cmd)

def approach_pulley():
    pass

def do_pulley():
    pass
        
# Initialization
rospy.init_node('phoenixbot_behavior_coordinator')

imu_sub = rospy.Subscriber("imu_data", Imu, imu_callback)
nav_result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, nav_callback)
simon_lights = rospy.Subscriber("light_sensors", Light, light_callback)

pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=3)
vel_pub = rospy.Publisher('move_base_controller/cmd_vel', Twist, queue_size=3)
nav_target = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
solenoid_pub = rospy.Subscriber("solenoid_commands", Solenoid, queue_size=1)

rospy.wait_for_service('/move_base/clear_costmaps')
clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

# Start
pose_pub.publish(get_pose("initial_pose"))
wait_for_start()

# Drive down the ramp
print("Driving to ramp...");
while not ramp:
    drive(0.75, 0.0)
print("Driving down ramp...");
while ramp:
    drive(0.75, 0.0)
print("Reached bottom of ramp");
drive(0.0, 0.0)

# Update the pose
print("Updating pose...")
rospy.sleep(1)
pose_pub.publish(get_pose("bottom_of_ramp"))
rospy.sleep(1)
clear_costmap()
rospy.sleep(1)

# Drive clear of the ramp
print("Clearing ramp")
drive(1.0, 0.0)
rospy.sleep(3)
drive(0.0, 0.0)

drive_to(get_waypoint("simon_approach"))
approach_simon()
do_simon()

drive_to(get_waypoint("pulley_approach"))
approach_pulley()
do_pulley()

print("FINISHED")

rospy.spin()

