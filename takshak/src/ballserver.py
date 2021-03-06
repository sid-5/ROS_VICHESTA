#!/usr/bin/env python2.7

from takshak.srv import ball
from takshak.srv import ballRequest
from takshak.srv import ballResponse
import cv2,math
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
    #data = rospy.wait_for_message('/camera/color/image_raw', Image)
    count = 0
    img = bridge.imgmsg_to_cv2(data.image, "bgr8") #desired_encoding='passthrough'
    # cv2.imwrite("ballserver.png", img)
    hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = hsvFrame[240,240]
    cv2.putText(img, "point", (240,240),
                  cv2.FONT_HERSHEY_SIMPLEX, 
                  1.0, (0, 255, 0))
    # rospy.loginfo(hsv)
    (rows,cols,channels) = img.shape
    width,height = 0,0
    #Balls mask
    ball_lower = np.array([165,50,60], np.uint8)
    ball_upper = np.array([169,255,255], np.uint8)
    ball_mask = cv2.inRange(hsvFrame, ball_lower, ball_upper)

    kernal = np.ones((7, 7), "uint8")
    ball_mask = cv2.dilate(ball_mask, kernal)

    res_ball = cv2.bitwise_and(img, img, mask = ball_mask)

    _, contours, hierarchy = cv2.findContours(ball_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.drawContours(res_ball, contours, -1, (0, 0, 255), 3)
    for pic, contour in enumerate(contours):
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
          # print(w,h,math.ceil((w+0.0)/(h+0.0)))
          # count+=math.ceil((w+0.0)/(h+0.0)) # when balls overlap
          count+=1
    cv2.imshow("Ball Detection",img)
    cv2.imwrite("Balldetection.png",img)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()
    rospy.loginfo("Returning count of balls: {}".format(count))
    return ballResponse(count)

def detect_ball_server():
    rospy.init_node('detect_ball_server', anonymous=True)
    s = rospy.Service('ball', ball, detect_ball)
    print "Detecting balls"
    rospy.spin()
    
if __name__ == "__main__":
    detect_ball_server()
