#!/usr/bin/env python3

import rospy
from robot_library.srv import *
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/r1/cmd_vel', Twist, queue_size=10)

def handle_set_velosities(req):
    # print(req)
    rospy.loginfo("There're new velosities: linear %s and angular %s"%(req.linear, req.angular))
    twist = Twist()
    twist.linear.x = req.linear
    twist.angular.z = req.angular
    pub.publish(twist)
    return SetVelositiesResponse(True)

def set_velosities_server():
    rospy.init_node('set_velosities_server')
    s = rospy.Service('set_velosities', SetVelosities, handle_set_velosities)
    rospy.loginfo("Ready to setting velosities.")
    rospy.spin()

if __name__ == "__main__":
    set_velosities_server()
