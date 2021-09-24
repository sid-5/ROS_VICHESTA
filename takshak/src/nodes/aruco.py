#!/usr/bin/env python3

# Import the ROS-Python package
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image

# Import numpy
import numpy as np

# Import opencv
import cv2

# Import aruco
import cv2.aruco as aruco

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge



# DEFINE THE PARAMETERS
# > For the number of the USB camera device
#   i.e., for /dev/video0, this parameter should be 0
USB_CAMERA_DEVICE_NUMBER = 0

# > For the size of the aruco marker, in meters
MARKER_SIZE = 0.151

# > For where to save images captured by the camera
#   Note: ensure that this path already exists
#   Note: images are only saved when a message is received
#         on the "request_save_image" topic.
SAVE_IMAGE_PATH = "/home/asc01/saved_camera_images/"

# > A flag for whether to display the images captured
SHOULD_SHOW_IMAGES = False



class TemplateArucoDetector:

    def __init__(self):
        
        # Initialise a publisher for the images
        self.image_publisher = rospy.Publisher("/global_namespace"+"/camera_image", Image, queue_size=10)

        # Initialise a subscriber for flagging when to save an image
        rospy.Subscriber("/global_namespace"+"/request_save_image", UInt32, self.requestSaveImageSubscriberCallback)
        # > For convenience, the command line can be used to trigger this subscriber
        #   by publishing a message to the "request_save_image" as follows:
        #
        # rostopic pub /global_namespace/request_save_image std_msgs/UInt32 "data: 1" 

        # Initialise varaibles for managing the saving of an image
        self.save_image_counter = 0
        self.should_save_image = False

        # Specify the details for camera to capture from

        # > For capturing from a USB camera:
        #   > List the content of /dev/video* to determine
        #     the number of the USB camera
        self.camera_setup = USB_CAMERA_DEVICE_NUMBER

        #   > Gstreamer for capture video
        #   > sensor-id=0 for CAM0 and sensor-id=1 for CAM1
        #   > This is not optimized, but it does work.
        #self.camera_setup = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=12/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, width = 800, height=600, format =BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

        # Initialise video capture from the camera
        self.cam=cv2.VideoCapture(self.camera_setup)

        # Initlaise the OpenCV<->ROS bridge
        self.cv_bridge = CvBridge()

        # Get the ArUco dictionary to use
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        # Create an parameter structure needed for the ArUco detection
        self.aruco_parameters = aruco.DetectorParameters_create()
        # > Specify the parameter for: corner refinement
        self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        # Specify the intrinsic parameters of the camera
        # > These parameters could either be hardcoded here;
        # > Or you can load then from a file that you may have
        #   saved during the calibration procedure.
        # > Note the that values hardcoded in this template
        #   may give meaningless results for your camera
        self.intrinic_camera_matrix = np.array( [[1726,0,1107] , [0,1726,788] , [0,0,1]], dtype=float)
        self.intrinic_camera_distortion  = np.array( [[ 5.5252e-02, -2.3523e-01, -1.0507e-04, -8.9834e-04, 2.4028e-01]], dtype=float)

        # Display the status
        rospy.loginfo("[TEMPLATE ARUCO DETECTOR] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(3.0), self.timerCallbackForPublishing)



    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        # Read the camera frame
        #rospy.loginfo('[TEMPLATE ARUCO DETECTOR] Now reading camera frame')
        return_flag , current_frame = self.cam.read()

        # Check if the camera frame was successfully read
        if (return_flag == True):
            # Convert the image to gray scale
            current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers from the frame
            aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = aruco.detectMarkers(current_frame_gray, self.aruco_dict, parameters=self.aruco_parameters)

            # Process any ArUco markers that were detected
            if aruco_ids is not None:
                # Display the number of markers found
                rospy.loginfo("[TEMPLATE ARUCO DETECTOR] ArUco marker found, len(aruco_ids) = " + str(len(aruco_ids)) )
                # Outline all of the markers detected found in the image
                current_frame_with_marker_outlines = aruco.drawDetectedMarkers(current_frame.copy(), aruco_corners_of_all_markers, aruco_ids, borderColor=(0, 220, 0))

                # Iterate over the markers detected
                for i_marker_id in range(len(aruco_ids)):
                    # Get the ID for this marker
                    this_id = aruco_ids[i_marker_id]
                    # Get the corners for this marker
                    corners_of_this_marker = aruco_corners_of_all_markers[i_marker_id]
                    # Estimate the pose of this marker
                    # > Note: ensure that the intrinsic parameters are
                    #   set for the specific camera in use.
                    this_rvec_estimate, this_tvec_estimate, _objPoints = aruco.estimatePoseSingleMarkers(corners_of_this_marker, MARKER_SIZE, self.intrinic_camera_matrix, self.intrinic_camera_distortion)
                    # Draw the pose of this marker
                    rvec = this_rvec_estimate[0]
                    tvec = this_tvec_estimate[0]
                    current_frame_with_marker_outlines = aruco.drawAxis(current_frame_with_marker_outlines, self.intrinic_camera_matrix, self.intrinic_camera_distortion, rvec, tvec, MARKER_SIZE)
                    rvec = rvec[0]
                    tvec = tvec[0]
                    
                    # At this stage, the variable "rvec" and "tvec" respectively
                    # describe the rotation and translation of the marker frame
                    # relative to the camera frame, i.e.:
                    # tvec - is a vector of length 3 expressing the (x,y,z) coordinates
                    #        of the marker's center in the coordinate frame of the camera.
                    # rvec - is a vector of length 3 expressinf the rotation of the marker's
                    #        frame relative to the frame of the camera.
                    #        This vector is an "axis angle" respresentation of the rotation.

                    # Compute the rotation matrix from the rvec using the Rodrigues
                    Rmat = cv2.Rodrigues(rvec)

                    # A vector expressed in the maker frame coordinates can now be
                    # rotated to the camera frame coordinates as:
                    # [x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}

                    # Note: the camera frame convention is:
                    # > z-axis points along the optical axis, i.e., straight out of the lens
                    # > x-axis points to the right when looking out of the lens along the z-axis
                    # > y-axis points to the down  when looking out of the lens along the z-axis

                    # Display the rvec and tvec
                    rospy.loginfo("[TEMPLATE ARUCO DETECTOR] for id = " + str(this_id) + ", tvec = [ " + str(tvec[0]) + " , " + str(tvec[1]) + " , " + str(tvec[2]) + " ]" )
                    #rospy.loginfo("[TEMPLATE ARUCO DETECTOR] for id = " + str(this_id) + ", rvec = [ " + str(rvec[0]) + " , " + str(rvec[1]) + " , " + str(rvec[2]) + " ]" )

                    # ============================================
                    # TO BE FILLED IN FOR WORLD FRAME LOCALISATION
                    # ============================================
                    # Based on the known location and rotation of
                    # marker relative to the world frame, compute
                    # an estimate of the camera's location within
                    # the world frame, and hence an estimate of
                    # robot's pose on which the camera is mounted. 
                    #
                    # ADD YOUR CODE HERE
                    #
                    # PUBLISH THE ESTIMATE OF THE ROBOT'S POSE
                    # FOR USE BY OTHER ROS NODES
                    #
                    # ============================================


            else:
                # Display that no aruco markers were found
                rospy.loginfo("[TEMPLATE ARUCO DETECTOR] No markers found in this image")

                current_frame_with_marker_outlines = current_frame_gray

            # Publish the camera frame
            #rospy.loginfo('[TEMPLATE ARUCO DETECTOR] Now publishing camera frame')
            self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame))

            # Save the camera frame if requested
            if (self.should_save_image):
                # Increment the image counter
                self.save_image_counter += 1
                # Write the image to file
                temp_filename = SAVE_IMAGE_PATH + "image" + str(self.save_image_counter) + ".jpg"
                cv2.imwrite(temp_filename,current_frame_with_marker_outlines)
                # Display the path to where the image was saved
                rospy.loginfo("[TEMPLATE ARUCO DETECTOR] Save camera frame to: " + temp_filename)
                # Reset the flag to false
                self.should_save_image = False

            # Display the camera frame if requested
            if (SHOULD_SHOW_IMAGES):
                rospy.loginfo("[TEMPLATE ARUCO DETECTOR] Now displaying camera frame")
                cv2.imshow("CAM 0", current_frame_with_marker_outlines)

        else:
            # Display an error message
            rospy.loginfo('[TEMPLATE ARUCO DETECTOR] ERROR occurred during self.cam.read()')



    # Respond to subscriber receiving a message
    def requestSaveImageSubscriberCallback(self, msg):
        rospy.loginfo("[TEMPLATE ARUCO DETECTOR] Request receieved to save the next image")
        # Set the flag for saving an image to true
        self.should_save_image = True



if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "template_aruco_detector"
    rospy.init_node(node_name)
    template_aruco_detector = TemplateArucoDetector()
    # Spin as a single-threaded node
    rospy.spin()

    # Release the camera
    template_aruco_detector.cam.release()
    # Close any OpenCV windows
    cv2.destroyAllWindows()
	
