#!/usr/bin/env python

import sys
from robot import *
import cv2
import numpy as np

if __name__ == "__main__":
    # print(get_encoders_client())
    robot = Robot()
    print(robot.getImage())

    exit(1)

