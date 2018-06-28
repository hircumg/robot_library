#!/usr/bin/env python

import rospy
from multi_robot_scenario.srv import *
from sensor_msgs.msg import LaserScan


laserScan = LaserScan()

def callback(msg):
    global laserScan
    laserScan = msg

def handle_get_laser(req):
    rospy.loginfo("New request for laser")
    return GetLaserResponse(laserScan)


def get_laser_server():
    rospy.init_node('get_laser_server',  anonymous=True)
    rospy.Subscriber("/r1/front_laser/scan", LaserScan, callback)
    s = rospy.Service('get_laser', GetLaser, handle_get_laser)
    print("Ready to transmitting laser data.")
    rospy.spin()

if __name__ == "__main__":
    get_laser_server()