#!/usr/bin/env python3

import socket
from time import sleep
import signal		
import sys		
import numpy as np				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2
import cv2.aruco as aruco
from cmath import atan # If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation
				# OpenCV Library
########################### GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()
pi = 3.14
ip = ""     #Enter IP address of laptop after connecting it to WIFI hotspot

##################### FUNCTION DEFINITIONS #######################

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
    
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
	arucoParameters = aruco.DetectorParameters_create()
	corners, ids, rejectedImgPoints = aruco.detectMarkers(current_frame, aruco_dict, parameters=arucoParameters)
    #vasee code
	image = aruco.drawDetectedMarkers(current_frame,corners,ids)     
	cv2.imshow('Display', image)
	cv2.waitKey(1)
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
	# print(f'{aruco_msg}')
	
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('usb_cam/image_rect', Image, callback)
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((ip, 8002)) 
		s.listen()
		conn, addr = s.accept()
		with conn:
			print(f"Connected by {addr}")
			while True:
				data = conn.recv(1024)
				print(counter)
				print(data)
				conn.sendall(str.encode(str(counter)))
				counter += 1
				sleep(1)
				# if counter == 10:
				# 	s.close()
				# 	break
	rospy.spin()
  
if __name__ == '__main__':
  main()
