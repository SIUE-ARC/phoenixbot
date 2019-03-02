#!/usr/bin/env python
import rospy
import numpy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped

covariance = [0.02, 0.0,  0.0, 0.0, 0.0, 0.0, 
              0.0,  0.02, 0.0, 0.0, 0.0, 0.0,
              0.0,  0.0,  0.02, 0.0, 0.0, 0.0,
              0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
              0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
              0.0,  0.0,  0.0, 0.0, 0.0, 0.0]


robots=set(['phoenixbot'])
transform = numpy.array([])
pose = None
def model_state_callback(msg):
    global pose
    for model, p in zip(msg.name, msg.pose):
        if model in robots:
            pose = p

def inital_pose_callback(msg):
    global transform
    initial_pose = numpy.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    current_pose = numpy.array([pose.position.x, pose.position.y, pose.position.z])

    # current_pose * transform = real_pose
    # transform = current_pose.inverse() * initalpose

    transform = -current_pose + initial_pose
    print(transform)


rospy.init_node('sim_beacon')
rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, inital_pose_callback)
pose_publisher = rospy.Publisher("/beacon_pose", PoseWithCovarianceStamped, queue_size=1)

r = rospy.Rate(6)
while not rospy.is_shutdown():
    m = PoseWithCovarianceStamped()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = "map"
    m.pose.pose = pose

    if transform.any():
        m.pose.pose.position.x += transform[0]
        m.pose.pose.position.y += transform[1]
        m.pose.pose.position.z += transform[2]

    m.pose.covariance = covariance
    pose_publisher.publish(m)
    r.sleep()

