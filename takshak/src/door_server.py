#!/usr/bin/env python

from takshak.srv import door_colour
from takshak.srv import door_colourRequest
from takshak.srv import door_colourResponse
import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import rospy


def detect_colour(img):
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    #red mask
    red_lower = np.array([176, 60, 60], np.uint8) #BGR
    red_upper = np.array([179, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    #yellow mask
    yellow_lower = np.array([28, 60, 60], np.uint8)
    yellow_upper = np.array([30, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    #blue mask
    blue_lower = np.array([100, 60, 60], np.uint8)
    blue_upper = np.array([103, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    #green mask
    green_lower = np.array([60, 60, 60], np.uint8)
    green_upper = np.array([65, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
              
    #purple mask
    purple_lower = np.array([146, 60, 60], np.uint8)
    purple_upper = np.array([148, 255, 255], np.uint8)
    purple_mask = cv2.inRange(hsvFrame, purple_lower, purple_upper)


    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((7, 7), "uint8")
      
    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    #res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
      
    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    #res_green = cv2.bitwise_and(imageFrame, imageFrame,mask = green_mask)
      
    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    #res_blue = cv2.bitwise_and(imageFrame, imageFrame,mask = blue_mask)

    # For yellow color
    yellow_mask = cv2.dilate(yellow_mask, kernal)

    # For purple color
    purple_mask = cv2.dilate(purple_mask, kernal)
    

    
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200):
          x, y, w, h = cv2.boundingRect(contour)
          imageFrame = cv2.rectangle(imageFrame, (x, y),(x + w, y + h),(0, 0, 255), 2)
          cv2.putText(imageFrame, "Red Colour", (cx, cy),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))
          answer[key] = "Red"
          rospy.loginfo("Returning red")
          return "Red"    
  
    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
              
            cv2.putText(imageFrame, "Green Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 255, 0))
            answer[key] = "Green"
            rospy.loginfo("Returning greeen")
            return "Green"
  
    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "Blue Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Blue"
            rospy.loginfo("Returning blue")
            return "Blue"

    # Creating contour to track yellow color
    contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "Yellow Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Yellow"
            rospy.loginfo("Returning Yellow")
            return "Yellow"

    # Creating contour to track purple color
    contours, hierarchy = cv2.findContours(purple_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "purple Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Purple" 
            rospy.loginfo("Returning purple")
            return "Purple"
	return "colour"


def handle_door_color(req):
	colour = ""
    img = bridge.imgmsg_to_cv2(req.image, "bgr8") #desired_encoding='passthrough'
    cv2.imwrite("door.png", img)
    colour = detect_colour()
    return door_colourResponse(colour)


def detect_door_colour():
    rospy.init_node('door_server')
    s = rospy.Service('door_colour', door_colour, handle_door_color)
    rospy.spin()
    
if __name__ == "__main__":
    detect_door_colour()
    cv2.destroyAllWindows()