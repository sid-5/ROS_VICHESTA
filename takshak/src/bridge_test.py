#!/usr/bin/env python

# SIMPLE SCRIPT TO TEST CV_BRIDGE PACKAGE
#
#- ON THE RASPI: roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true
#

import sys
import cv2.aruco as aruco
import numpy as np
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#--- Define our Class
class image_converter:

    def __init__(self):
        #--- Publisher of the edited frame
        self.image_pub = rospy.Publisher("image_topic",Image,queue_size=1)

        #--- Subscriber to the camera flow
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

	
    def findArucoMarkers(self, img, draw=True):
    	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    	arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    	arucoParam = aruco.DetectorParameters_create()
    	bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    	rospy.loginfo(ids)
        cv2.putText(img, ids, (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, [0,0,200], 5)
    	if draw:
        	aruco.drawDetectedMarkers(img, bboxs) 
    		return [bboxs, ids]
    	


    def callback(self,data):  #--- Callback function
    
        #--- Read the frame and convert it using bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough') #desired_encoding='passthrough'
        except CvBridgeError as e:
            print(e)

        #--- If a valid frame is received, draw a circle and write HELLO WORLD
        (rows,cols,channels) = cv_image.shape

        if cols > 20 and rows > 20:
            self.findArucoMarkers(cv_image)
            #--- Circle
            cv2.circle(cv_image, (500,500), 200, 255)
            
            #--- Text
            text = "HELLO WORLD"
            cv2.putText(cv_image, text, (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, [0,0,200], 5)


        #--- Optional: show the image on a window (comment this for the Raspberry Pi)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        #--- Publish the modified frame to a new topic
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    ic = image_converter()
    
    #--- Initialize the ROS node
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
