#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import Pose2D
import math

aruco_msg = Pose2D()
x_goals = [0,0,0,0,0,0,0,0,0,0,0,0,0]
y_goals = [0,0,0,0,0,0,0,0,0,0,0,0,0]
theta_goals = [0,0,0,0,0,0,0,0,0,0,0,0,0]

def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('/usb_cam/image_raw', Image, callback)
	
	rate = rospy.Rate(100)
	Count = 0
	Error = Pose2D
	limit = len(x_goals)

	while limit < 5:
		limit = len(x_goals)

	while not rospy.is_shutdown():
		inverse_kinematics(Count,limit)
		print(f' location {aruco_msg}')
	rospy.spin()

def callback(data):
	br = CvBridge()
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParameters = aruco.DetectorParameters_create()
	(corners, ids, rejectedImgPoints) =  aruco.detectMarkers(current_frame, aruco_dict, parameters=arucoParameters)
	image = aruco.drawDetectedMarkers(current_frame,corners,ids)     
	cv2.imshow('Display', image)
	cv2.waitKey(1)
	aruco_feedback_Cb()

def aruco_feedback_Cb():
	global aruco_msg
	aruco_msg.x = 250
	aruco_msg.y = 250
	aruco_msg.theta = 0

def inverse_kinematics(Count,limit):
	Error = Pose2D()
	Error.x = x_goals[Count]-aruco_msg.x
	Error.y = y_goals[Count]-aruco_msg.y
	Error.theta = theta_goals[Count]-aruco_msg.theta

	if abs(Error.theta)<0.05  and abs(Error.x)<5 and abs(Error.y)<5:
			motion_msg =0
			if Count< (limit-1):
				Count+=1
	else:
		x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
		y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))

		motion_msg.x= (2*x/3-Error.theta*5)*0.5
		motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*5)*0.5
		motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*5)*0.5

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass




#!/usr/bin/env python3

######################## IMPORT MODULES ##########################
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2
import cv2.aruco as aruco				# OpenCV Library
from cmath import atan # If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation
from geometry_msgs.msg import Point
import math
import numpy as np
import socket
import sys
from time import sleep
############################ GLOBALS #############################

#location of bot
aruco_msg = Pose2D()

#goal points
x_goals = []
y_goals = []
theta_goals = []


# to send speed
speed = Point()
# will remove in future
counter = 1

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

# ## cleanup and signal_handler are clear up fucntion when program ends
# def signal_handler(sig, frame):
#     print('Clean-up !')
#     cleanup()
#     sys.exit(0)
# def cleanup():
#     s.close()
#     print("cleanup done")

#will remove in future
# def home_pos():
# 	aruco_msg.x = 250
# 	aruco_msg.y = 250
# 	aruco_msg.theta = 0	

#gets the co-ordinates of the bot
def aruco_feedback_Cb():

# 	# x_axis = (corners[0][0][1][0]-corners[0][0][0][0])
# 	# y_axis = (corners[0][0][0][1]-corners[0][0][1][1])
# 	# angle = atan(y_axis/x_axis)
# 	# if x_axis<0:
# 	# 	if y_axis >0:
# 	# 		angle += pi
# 	# 	elif y_axis  <0:
# 	# 		angle -= pi
# 	# aruco_msg.x =(corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])*0.25
# 	# aruco_msg.y =(corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1])*0.25
# 	# aruco_msg.theta = angle.real
	global aruco_msg
	aruco_msg.x = 250
	aruco_msg.y = 250
	aruco_msg.theta = 0

# convets the ros image to opencv and detects aruco marker	
def callback(data):
    
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
    #need to add exact code aruco marker detecitonn 
    #its is detecting random aruco marker
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParameters = aruco.DetectorParameters_create()
	(corners, ids, rejectedImgPoints) =  aruco.detectMarkers(current_frame, aruco_dict, parameters=arucoParameters)
	image = aruco.drawDetectedMarkers(current_frame,corners,ids)     
	cv2.imshow('Display', image)
	cv2.waitKey(1)
	aruco_feedback_Cb()

#gets inverse matrix 
# def inverse_kinematics(Error,motion_msg):

# 	x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
# 	y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))

# 	motion_msg.x= (2*x/3-Error.theta*5)*0.5
# 	motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*5)*0.5
# 	motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*5)*0.5
# 	return motion_msg

#serialize the array
# def output(msg):
#     speed = msg
#     array = np.array([speed.x,speed.y,speed.z], dtype=np.float32)
#     serialized_array = array.tobytes()
#     loop(serialized_array)

# def loop(serialized_array):
#     global counter 
#     data = conn.recv(1024)
#     print("works")
#     conn.send(serialized_array)
#     counter += 1
#     sleep(1)
# 	#need to modify the condtion
#     if counter == 10:
#        s.close()
   

def main():

	global aruco_msg
	# home_pos()
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('usb_cam/image_rect', Image, callback)


	motion_msg = Point()
	
	rate = rospy.Rate(100)
	Count = 0
	Error = Pose2D()
	limit = len(x_goals)

	while limit < 5:
		limit = len(x_goals)

	while():
	
		Error.x = x_goals[Count]-aruco_msg.x
		Error.y = y_goals[Count]-aruco_msg.y
		Error.theta = theta_goals[Count]-aruco_msg.theta
		
		if abs(Error.theta)<0.05  and abs(Error.x)<5 and abs(Error.y)<5:
			motion_msg.x= 0
			motion_msg.y= 0
			motion_msg.z= 0
			# output(motion_msg)
			
			
			print(Count)
			if Count< (limit-1):
				Count+=1
				
		# else:
			# motion_msg = inverse_kinematics(Error,motion_msg)
			
		print(f"{motion_msg}")
		# output(motion_msg)
		# rate.sleep()
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass