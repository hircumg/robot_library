#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from rosgraph_msgs.msg import Clock
from robot_library.srv import *
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

SIMULATION_TIME = None

def time() -> float:
    """
    Function returns time from gazebo simulation, used to override standart python function
    """
    return SIMULATION_TIME

def sleep(secs) -> None:
    """
    Function stops executing program for seconds in simulation time
    """
    start_time = time()
    while time() - start_time < secs:
        pass


def simulation_time_callback(data):
    # print("we are updating something {}".format([data.clock.secs, data.clock.nsecs]))
    global SIMULATION_TIME
    # converting compound time object into float with seconds
    SIMULATION_TIME = data.clock.secs + data.clock.nsecs / 1000000000

rospy.init_node("clock_listener", anonymous=True)
rospy.Subscriber("/clock", Clock, simulation_time_callback)

class Robot:
    """Library class for working with pioneer-3dx in gazebo"""
    _pkg_name = "/robot_lib/"

    def __init__(self):
        pass

    def time(self) -> float:
        return time()

    def sleep(self, sec):
        sleep(sec)

    def __set_velosities_client(self, linear_vel, angular_vel):
        rospy.wait_for_service(self._pkg_name + 'set_velosities')
        try:
            set_velosities = rospy.ServiceProxy(self._pkg_name +  'set_velosities', SetVelosities)
            result = set_velosities(linear_vel, angular_vel)
            return result.isSuccess
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False


    def __get_laser_client(self):

        rospy.wait_for_service(self._pkg_name +  'get_laser')
        try:
            get_laser = rospy.ServiceProxy(self._pkg_name +  'get_laser', GetLaser)
            result = get_laser(True)
            return result.laser
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def __get_direction_client(self):
        rospy.wait_for_service(self._pkg_name +  'get_direction')
        try:
            get_direction = rospy.ServiceProxy(self._pkg_name +  'get_direction', GetDirection)
            result = get_direction(True)
            return result.direction
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def __get_camera_client(self):
        rospy.wait_for_service(self._pkg_name +  'get_camera')
        try:
            get_camera = rospy.ServiceProxy(self._pkg_name + 'get_camera', GetCamera)
            result = get_camera(True)
            return result.image
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def __get_encoders_client(self):
        rospy.wait_for_service(self._pkg_name +  'get_encoders')
        try:
            get_encoders = rospy.ServiceProxy(self._pkg_name +  'get_encoders', GetEncoders)
            result = get_encoders(True)
            # print(result)
            return {'left': result.left, 'right': result.right}
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)



    def getDirection(self):
        """:return direction of robot"""
        return self.__get_direction_client()

    def getEncoders(self):
        """:return left and right encoders of the robot like dict"""
        return self.__get_encoders_client()

    def getLaser(self):
        """:returns laser values, time stamp for current values, angle of laser, 
        and angle increment for calculate values"""
        _laser = self.__get_laser_client()
        laser = {"time_stamp" : _laser.header.stamp.nsecs, "angle" : (_laser.angle_max * 2),
                 "angle_increment" : _laser.angle_increment, "values" : _laser.ranges}
        return laser

    def setVelosities(self, linear_vel, angular_vel):
        """This function try to set up linear and angular velosities
        to the robot in gazebo simulation
        :return true if succseed"""
        return self.__set_velosities_client(linear_vel, angular_vel)

    def getImage(self):
        """:return camera image like openCV frame"""
        _frame = self.__get_camera_client()
        cv_image = CvBridge().imgmsg_to_cv2(_frame, desired_encoding="passthrough")
        return cv_image
