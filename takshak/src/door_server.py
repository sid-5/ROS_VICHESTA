#!/usr/bin/env python

from takshak.srv import door_colour
from takshak.srv import door_colourRequest
from takshak.srv import door_colourResponse
import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import rospy