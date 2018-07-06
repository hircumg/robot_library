#!/usr/bin/env python

import rospy
from robot_library.srv import *
from sensor_msgs.msg import Image


image = Image()

def callback(msg):
    global image
    image = msg


def handle_get_camera(req):
    rospy.loginfo("New request for camera image")
    return GetCameraResponse(image)


def get_camera_server():
    rospy.init_node('get_camera_server',  anonymous=True)
    rospy.Subscriber("/r1/front_camera/image_raw", Image, callback)
    s = rospy.Service('get_camera', GetCamera, handle_get_camera)
    print("Ready to transmitting camera information.")
    rospy.spin()

if __name__ == "__main__":
    get_camera_server()