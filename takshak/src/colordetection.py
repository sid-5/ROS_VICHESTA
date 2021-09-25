#!/usr/bin/env python
import numpy as np
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() 
  
def color():
        
        image_sub = rospy.Subscriber("/camera/color/image_raw",Image,callback)
  
def callback(data):  #--- Callback function
    
        #--- Read the frame and convert it using bridge
        try:
            imageFrame = bridge.imgmsg_to_cv2(data, "bgr8") #desired_encoding='passthrough'
            #while(1):
      
            # Reading the video from the
            # webcam in image frames
          
            # Convert the imageFrame in 
            # BGR(RGB color space) to 
            # HSV(hue-saturation-value)
            # color space
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
          
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
            blue_lower = np.array([94, 80, 2], np.uint8)
            blue_upper = np.array([120, 255, 255], np.uint8)
            blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
              
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
                if(area > 300):
                    x, y, w, h = cv2.boundingRect(contour)
                    imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                               (x + w, y + h), 
                                               (0, 0, 255), 2)
                      
                    cv2.putText(imageFrame, "Red Colour", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                (0, 0, 255))    
          
            # Creating contour to track green color
            _, contours, hierarchy = cv2.findContours(green_mask,
                                                   cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_SIMPLE)
              
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 300):
                    x, y, w, h = cv2.boundingRect(contour)
                    imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                               (x + w, y + h),
                                               (0, 255, 0), 2)
                      
                    cv2.putText(imageFrame, "Green Colour", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                1.0, (0, 255, 0))
          
            # Creating contour to track blue color
            _,  contours, hierarchy = cv2.findContours(blue_mask,
                                                   cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_SIMPLE)
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 300):
                    x, y, w, h = cv2.boundingRect(contour)
                    imageFrame = cv2.rectangle(imageFrame, (x, y),
                                               (x + w, y + h),
                                               (255, 0, 0), 2)
                      
                    cv2.putText(imageFrame, "Blue Colour", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (255, 0, 0))
                      
            # Program Termination
            cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)

#--------------- MAIN LOOP
def main(args):
    color()
    
    #--- Initialize the ROS node
    rospy.init_node('color_detect', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
