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

	
    def findArucoMarkers(self, img, draw=False):
    	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    	arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    	arucoParam = aruco.DetectorParameters_create()
    	bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    	if draw:
        	aruco.drawDetectedMarkers(img, bboxs, ids) 
    	return [bboxs, ids]

    def colorDetect(self, imageFrame, x_cor, y_cor):
        #--- Read the frame and convert it using bridge          
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        colors = {}
        # Set range for red color and 
        # define mask
        
        red_lower = np.array([0, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
      
        # Set range for green color and 
        # define mask
        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
      
        # Set range for blue color and
        # define mask
        blue_lower = np.array([1, 80, 2], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
        '''
        #red mask
        red_lower = np.array([0, 0, 90], np.uint8)
        red_upper = np.array([0, 0, 155], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        #blue mask
        blue_lower = np.array([90, 60, 20], np.uint8)
        blue_upper = np.array([155, 105, 40], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        #green mask
        green_lower = np.array([5, 80, 30], np.uint8)
        green_upper = np.array([40, 100, 40], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        '''

        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")
          
        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                  mask = red_mask)
          
        # For green color
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                    mask = green_mask)
          
        # For blue color
        blue_mask = cv2.dilate(blue_mask, kernal)
        res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                                   mask = blue_mask)
       
        # Creating contour to track red color
        _, contours, hierarchy = cv2.findContours(red_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
          
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 200):
                x, y, w, h = cv2.boundingRect(contour)
                if(x<x_cor<(x+w) and y<y_cor):
                    imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                           (x + w, y + h), 
                                           (0, 0, 255), 2)
                  
                    #cv2.putText(imageFrame, "Red Colour:a{}x{}y{}".format(area,x,y), (x, y),
                    #        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    #        (0, 0, 255))
                    cv2.putText(imageFrame, "{}".format(np.array(cv2.mean(imageFrame[y:y+h,x:x+w])).astype(np.uint8)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0))
                    colors["red"] = (x,x+w)   
      
        # Creating contour to track green color
        _, contours, hierarchy = cv2.findContours(green_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
          
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 200):
                x, y, w, h = cv2.boundingRect(contour)
                if(x<x_cor<(x+w) and y_cor>y):    
                    imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                           (x + w, y + h),
                                           (0, 255, 0), 2)
                  
                    #cv2.putText(imageFrame, "Green Colour:a{}x{}y{}".format(area,x,y), (x, y),
                    #        cv2.FONT_HERSHEY_SIMPLEX, 
                    #        1.0, (0, 255, 0))
                    cv2.putText(imageFrame, "{}".format(np.array(cv2.mean(imageFrame[y:y+h,x:x+w])).astype(np.uint8)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0))
                    colors["green"] = (x,x+w)
      
        # Creating contour to track blue color
        _,  contours, hierarchy = cv2.findContours(blue_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 200):
                x, y, w, h = cv2.boundingRect(contour)
                if(x<x_cor<(x+w) and y_cor>y):
                    imageFrame = cv2.rectangle(imageFrame, (x, y),
                                           (x + w, y + h),
                                           (255, 0, 0), 2)
                  
                    #cv2.putText(imageFrame, "Blue Colour:a{}x{}y{}".format(area,x,y), (x, y),
                    #        cv2.FONT_HERSHEY_SIMPLEX,
                    #        1.0, (255, 0, 0))
                    cv2.putText(imageFrame, "{}".format(np.array(cv2.mean(imageFrame[y:y+h,x:x+w])).astype(np.uint8)), (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0))
                    
                    colors["blue"] = x+(w/2)
                  
        '''# Program Termination
        cv2.imshow("Color Detection in Real-TIme", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()'''

    	


    def callback(self,data):  #--- Callback function
    
        #--- Read the frame and convert it using bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") #desired_encoding='passthrough'
        except CvBridgeError as e:
            print(e)

        #--- If a valid frame is received
        (rows,cols,channels) = cv_image.shape
        width,height = 0,0

        if cols > 20 and rows > 20:
            arucofound = self.findArucoMarkers(cv_image)
             # loop through all the markers and augment each one
            if  len(arucofound[0])!=0:
                for bbox, id in zip(arucofound[0], arucofound[1]):
                    width = (bbox.item(0,0,0)+bbox.item(0,1,0))/2
                    height = (bbox.item(0,0,1)+bbox.item(0,2,1))/2
                    #cv2.putText(cv_image, id, (width[0], height[1]),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))
                    rospy.loginfo("""mean width pos {} 
                                     mean height pos {}
                                     has id{}""".format(width,height,id.item(0)))
                self.colorDetect(cv_image, width,(height-50))
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
