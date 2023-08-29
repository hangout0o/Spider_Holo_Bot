#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ HB#1310]
# Author List:		[ Vaseekaran E R,Mohamed Asslam B,Shriharshini R,Hurath R]
# Filename:		feedback.py
# Functions:
#			[ main ,pose ,callback]
# Nodes:		overhead_cam/image_raw --> Subscriber
#               detected_aruco --> Publisher


######################## IMPORT MODULES ##########################

import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2
import cv2.aruco as aruco				# OpenCV Library
import math				
from cmath import atan # If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()
pi = 3.14
##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code
def pose(corners):
	global aruco_msg,aruco_publisher,angle
	x_axis = (corners[0][0][1][0]-corners[0][0][0][0])
	y_axis = (corners[0][0][0][1]-corners[0][0][1][1])
	angle = atan(y_axis/x_axis)
	if x_axis<0:
		if y_axis >0:
			angle += pi
		elif y_axis  <0:
			angle -= pi
	aruco_msg.x =(corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])*0.25
	aruco_msg.y =(corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1])*0.25
	aruco_msg.theta = angle.real
	aruco_publisher.publish(aruco_msg)

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
    
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParameters = aruco.DetectorParameters_create()
	(corners,dfd,gd) =  aruco.detectMarkers(current_frame, aruco_dict, parameters=arucoParameters)
	cv2.imshow("okay",current_frame)
	cv2.waitKey(1)
	pose(corners)
	
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()
