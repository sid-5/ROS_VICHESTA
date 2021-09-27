
#!/usr/bin/env python

from ros_essentials_cpp.srv import AddTwoInts
from ros_essentials_cpp.srv import AddTwoIntsRequest
from ros_essentials_cpp.srv import AddTwoIntsResponse

import rospy

def handle_aruco_color(req):
	aruco_ids=[]
	color_ids=[]
    # store here
    return aruco_ids,color_ids

def add_aruco_color_server():
    rospy.init_node('aruco_color_server')
    s = rospy.Service('aruco_color', AddTwoInts, handle_aruco_color)
    rospy.spin()
    
if __name__ == "__main__":
    add_aruco_color_server()
