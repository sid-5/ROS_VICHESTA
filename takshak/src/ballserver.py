#!/usr/bin/env python

from ball.srv import ball
from ball.srv import ballRequest
from ball.srv import ballResponse
import cv2
import cv2.aruco as aruco
from matplotlib import pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() 


def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if(M['m00']!=0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx,cy


def detect_ball(data):
    count = 0
    img = bridge.imgmsg_to_cv2(data, "bgr8") #desired_encoding='passthrough'
    hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    (rows,cols,channels) = img.shape
    width,height = 0,0

    #Balls mask
    ball_lower = np.array([160,50,50], np.uint8)
    ball_upper = np.array([165,255,255], np.uint8)
    ball_mask = cv2.inRange(hsvFrame, ball_lower, ball_upper)


    kernal = np.ones((7, 7), "uint8")


    ball_mask = cv2.dilate(ball_mask, kernal)

    res_ball = cv2.bitwise_and(img, img, mask = ball_mask)

    contours, hierarchy = cv2.findContours(ball_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.drawContours(res_ball, contours, -1, (0, 0, 255), 3)
    plt.imshow(output)
    for pic, contour in enumerate(contours):
      counter += 1
      area = cv2.contourArea(contour)
      cx, cy = get_contour_center(contour)
      if(area > 100):
          x, y, w, h = cv2.boundingRect(contour)
          img = cv2.rectangle(img, (x, y), 
                                      (x + w, y + h),
                                      (0, 255, 0), 2)
          cv2.putText(img, "ball", (cx, cy),
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      1.0, (0, 255, 0))
    cv2.imshow("Ball Detection",img)
    rospy.loginfo("Returning {}".format(count))
    return ballResponse(count)

def detect_ball_server():
    rospy.init_node('detect_ball_server')
    s = rospy.Service('ball', ball, detect_ball)
    print "Detecting balls"
    rospy.spin()
    
if __name__ == "__main__":
    detect_ball_server()