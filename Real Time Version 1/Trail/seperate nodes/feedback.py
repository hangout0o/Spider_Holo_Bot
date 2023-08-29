#!/usr/bin/env python3

######################## IMPORT MODULES ##########################

import numpy as np				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2
import cv2.aruco as aruco				# OpenCV Library
from cmath import atan # If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
pi = 3.14
marker_coords = np.array([[0, 0], [500, 0], [500, 500], [0, 500]], dtype=np.float32)


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code
def pose(corners,ids):
	aruco_msg = Pose2D()
	bot = corners[np.where(ids == 15)[0][0]][0]
	x_axis = (bot[1][0]-bot[0][0])
	y_axis = (bot[0][1]-bot[1][1])
	angle = atan(y_axis/x_axis)
	if x_axis<0:
		if y_axis >0:
			angle += pi
		elif y_axis  <0:
			angle -= pi
	aruco_msg.x =(bot[0][0]+bot[1][0]+bot[2][0]+bot[3][0])*0.25
	aruco_msg.y =(bot[0][1]+bot[1][1]+bot[2][1]+bot[3][1])*0.25
	aruco_msg.theta = angle.real
	return aruco_msg

def crop(corners,ids,current_frame):
	
	marker_coords_detected = []
	marker_coords_detected.append(corners[np.where(ids == 4)[0][0]][0][3])
	marker_coords_detected.append(corners[np.where(ids == 8)[0][0]][0][0])
	marker_coords_detected.append(corners[np.where(ids == 10)[0][0]][0][1])
	marker_coords_detected.append(corners[np.where(ids == 12)[0][0]][0][2])
	marker_coords_detected = np.array(marker_coords_detected, dtype=np.float32)
	M = cv2.getPerspectiveTransform(marker_coords_detected, marker_coords)
	img_proj = cv2.warpPerspective(current_frame, M, (500, 500))
	return img_proj

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	# rospy.loginfo("receiving camera frame")
	current_frame 		= br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	# current_frame 	= cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	aruco_dict		= aruco.Dictionary_get(aruco.DICT_4X4_50)
	arucoParameters = aruco.DetectorParameters_create()

	corners, ids, _ = aruco.detectMarkers(current_frame , aruco_dict, parameters=arucoParameters)

	if ids is not None:
		# corners=corners
		new_frame = crop(corners,ids,current_frame)
		arucoParameters.errorCorrectionRate = 1
		arucoParameters.adaptiveThreshConstant = 10
		corners, ids, _ = aruco.detectMarkers(new_frame , aruco_dict, parameters=arucoParameters)
		# print(ids)
		# if _ is not None:
		# 	print(_[1])

		
		aruco_msg = pose(corners,ids)
		length = f"{aruco_msg.x}, {aruco_msg.y}"
		cv2.putText(new_frame,length, (20,40), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
		length = f"{aruco_msg.theta}"
		cv2.putText(new_frame,length, (20,60), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
		image = aruco.drawDetectedMarkers(new_frame,corners,ids)  
		cv2.imshow('Display',image)
		cv2.waitKey(1)
	else:
		print("Aruco markers not detected in image.")



	

def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('usb_cam/image_rect', Image, callback)
	rospy.spin()

if __name__ == '__main__':
  main()
