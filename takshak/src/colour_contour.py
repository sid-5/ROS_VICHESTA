#!/usr/bin/env python
import numpy as np
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() 
dictt = {}
answer = {}


#red mask
red_lower = np.array([0, 99, 36], np.uint8) #BGR
red_upper = np.array([2, 106, 60], np.uint8)
red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

#yellow mask
yellow_lower = np.array([53, 99, 40], np.uint8)
yellow_upper = np.array([56, 106, 80], np.uint8)
yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

#blue mask
blue_lower = np.array([204, 60, 30], np.uint8)
blue_upper = np.array([208, 70, 70], np.uint8)
blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

#green mask
green_lower = np.array([98, 85, 30], np.uint8)
green_upper = np.array([100, 95, 45], np.uint8)
green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
          
#purple mask
purple_lower = np.array([275, 55, 28], np.uint8)
purple_upper = np.array([280, 65, 40], np.uint8)
purple_mask = cv2.inRange(hsvFrame, purple_lower, purple_upper)

#Balls mask
ball_lower = np.array([333, 55, 25], np.uint8)
ball_upper = np.array([335, 60, 95], np.uint8)
ball_mask = cv2.inRange(hsvFrame, ball_lower, ball_upper)

# Morphological Transform, Dilation
# for each color and bitwise_and operator
# between imageFrame and mask determines
# to detect only that particular color
kernal = np.ones((5, 5), "uint8")
  
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

# For ball
ball_mask = cv2.dilate(ball_mask, kernal)


def get_contour_center():
	M = cv2.moments(contour)
	cx = -1
	cy = -1
	if(M['m00']!=0):
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
	return cx,cy


def aruco_detect(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_250)
	arucoParam = aruco.DetectorParameters_create()
	bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
	#if draw:
    	#aruco.drawDetectedMarkers(img, bboxs, ids)
	return [bboxs, ids]


def colour_detect(imageFrame, width_s, width_e, height, key):
	# Creating contour to track red color
    _, contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200 and width_s<cx<width_e and cy<height):
        	x, y, w, h = cv2.boundingRect(contour)
        	imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h), 
                                       (0, 0, 255), 2)
            cv2.putText(imageFrame, "Red Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))
            answer[key] = "Red"    
  
    # Creating contour to track green color
    _, contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200 and width_s<cx<width_e and cy<height):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
              
            cv2.putText(imageFrame, "Green Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 255, 0))
           	answer[key] = "Green" 
  
    # Creating contour to track blue color
    _,  contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200 and width_s<cx<width_e and cy<height):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "Blue Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Blue" 

    # Creating contour to track yellow color
    _,  contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200 and width_s<cx<width_e and cy<height):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "Yellow Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Yellow" 

    # Creating contour to track purple color
    _,  contours, hierarchy = cv2.findContours(purple_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200 and width_s<cx<width_e and cy<height):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "purple Colour", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Purple" 

    # Creating contour to track balls
    _,  contours, hierarchy = cv2.findContours(ball_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cx, cy = get_contour_center(contour)
        if(area > 200 and width_s<cx<width_e and cy<height):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "ball", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
            answer[key] = "Ball" 

def callback(data):  #--- Callback function
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8") #desired_encoding='passthrough'
        except CvBridgeError as e:
            print(e)

        #--- If a valid frame is received
        (rows,cols,channels) = cv_image.shape
        width,height = 0,0
		if cols > 20 and rows > 20:
		            arucofound = findArucoMarkers(cv_image)
		             # loop through all the markers and augment each one
		            if  len(arucofound[0])!=0:
		                for bbox, id in zip(arucofound[0], arucofound[1]):
		                	dcit[id.item(0)] = (bbox.item(0,0,0),bbox.item(0,1,0))
		                    width = (bbox.item(0,0,0)+bbox.item(0,1,0))/2
		                    height = (bbox.item(0,0,1)+bbox.item(0,2,1))/2
		                    #cv2.putText(cv_image, id, (width[0], height[1]),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))
		                    rospy.loginfo("""mean width pos {} 
		                                     mean height pos {}
		                                     has id{}""".format(width,height,id.item(0)))
		                for key, value in dictt.items():
		                	colorDetect(cv_image, value[0], value[1],(height-200), key)
		        cv2.imshow("Image window", cv_image)
		        cv2.waitKey(3)
        


def main(args):
    image_sub = rospy.Subscriber("/camera/color/image_raw",Image,callback)
	imageFrame = bridge.imgmsg_to_cv2(data, "bgr8") #desired_encoding='passthrough'
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    
    #--- Initialize the ROS node
    rospy.init_node('cv_detect', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)