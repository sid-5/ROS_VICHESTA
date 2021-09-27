#!/usr/bin/env python

from colour_aruco.srv import colour_aruco
from colour_aruco.srv import colour_arucoRequest
from colour_aruco.srv import colour_arucoResponse

import rospy

def handle_aruco_color(req):
	aruco_ids=[]
	color_ids=[]
    <>
    return aruco_ids,color_ids

def add_aruco_color_server():
    rospy.init_node('aruco_color_server')
    s = rospy.Service('aruco_color', colour_aruco, handle_aruco_color)
    rospy.spin()
    
if __name__ == "__main__":
    add_aruco_color_server()
