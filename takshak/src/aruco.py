#!/usr/bin/env python

import sys
import cv2.aruco as aruco
import numpy as np
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#--- Define our Class
class Aruco:

    def __init__(self):

        #--- Subscriber to the camera flow
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

	
    def findArucoMarkers(self, img, draw=True):
    	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    	arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    	arucoParam = aruco.DetectorParameters_create()
    	bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    	if draw:
        	aruco.drawDetectedMarkers(img, bboxs, ids) 
    	return [bboxs, ids]
    	


    def callback(self,data):  #--- Callback function
    
        #--- Read the frame and convert it using bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") #desired_encoding='passthrough'
        except CvBridgeError as e:
            print(e)

        #--- If a valid frame is received
        (rows,cols,channels) = cv_image.shape

        if cols > 20 and rows > 20:
            arucofound = self.findArucoMarkers(cv_image)
             # loop through all the markers and augment each one
            if  len(arucofound[0])!=0:
                for bbox, id in zip(arucofound[0], arucofound[1]):
                    rospy.loginfo("""mean width pos {} 
                        mean height pos {}
                        has id{}""".format((bbox[0]+bbox[1])/2,(bbox[0]+bbox[2])/2,id))
                    time.sleep(0.5)
            
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

       
#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    ic = Aruco()
    
    #--- Initialize the ROS node
    rospy.init_node('aruco_detect', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
