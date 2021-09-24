#!/usr/bin/env python


import rospy
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge

bridge = CvBridge()

def main():
	rospy.init_node('cv_node', anonymous=True)
        sub = rospy.Subscriber('/camera/color/image_raw',Image,callback)
	rospy.loginfo("suscribed")

def callback(ros_pics):
      	try: 
	    rospy.loginfo("entered callback")
            my_image = bridge.imgmsg_to_cv2(ros_pics, encoding = "passthrough")

        except:
            rospy.loginfo("CvBridge could not convert images from realsense to opencv")
       	    return 0;

    	cv2.imshow("Robot Image", cv_image)
        #cv2.imshow('Image HSV', hsv_image)
        #cv2.imshow('Image Gray', gray)
        frame = np.array(cv_image, dtype=np.uint8)
	rospy.loginfo(frame)
	



if __name__ == '__main__':
        try:
            main()
	    rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Test finished.")
