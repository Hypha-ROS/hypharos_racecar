#!/usr/bin/env python

# File: opencv_test.py
# Abstract: Capture Video stream then publishing it through ROS topic
# Author: HaoChih, LIN
# License: BSD (3-clause version)
# Email: hypha.ros@gmil.com
# Date: 2017/06/18

## Import libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## ROS Python Init
rospy.init_node("opencv_ros_python")
imagePub = rospy.Publisher("image_gray",Image, queue_size=10)
cvBridge = CvBridge()

## OpenCV Capture
cap = cv2.VideoCapture(0)

## While loop
while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    cv_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',cv_gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Convert cvImg to ROS topic
    try:
        imagePub.publish(cvBridge.cv2_to_imgmsg(cv_gray, "mono8"))
    except CvBridgeError as e:
        print(e)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
