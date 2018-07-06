#!/usr/bin/env python

import rospy
from robot_library.srv import *
from sensor_msgs.msg import JointState


encoders = []

def callback(msg):
    global encoders
    if len(msg.position) > 3:
        encoders = msg.position
        # rospy.loginfo("encoders: left: %s and right: %s" %(encoders[2], encoders[3]))


def handle_get_encoders(req):
    rospy.loginfo("New request for encoders of robot")
    return GetEncodersResponse(encoders[2],encoders[3])


def get_encoders_server():
    rospy.init_node('get_encoders_server',  anonymous=True)
    rospy.Subscriber("/r1/joint_states", JointState, callback)
    s = rospy.Service('get_encoders', GetEncoders, handle_get_encoders)
    print("Ready to transmitting encoders.")
    rospy.spin()

if __name__ == "__main__":
    get_encoders_server()