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
x_goals = [0,0,0,0,0,0,0,0]
y_goals =  [0,0,0,0,0,0,0,0]
theta_goals = [0,0,0,0,0,0,0,0]

#ip address 
ip = "192.168.150.171"  

#socket lines
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((ip, 8002))
s.listen()
conn, addr = s.accept()

# to send speed
speed = Point()
# will remove in future
counter = 1

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

## cleanup and signal_handler are clear up fucntion when program ends
def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)
def cleanup():
    s.close()
    print("cleanup done")

#will remove in future
def home_pos():
	aruco_msg.x = 250
	aruco_msg.y = 250
	aruco_msg.theta = 0	

#gets the co-ordinates of the bot
# convets the ros image to opencv and detects aruco marker	
def callback(data):

	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	# rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	#need to add exact code aruco marker deteciton
	#its is detecting random aruco marker
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParameters = aruco.DetectorParameters_create()
	(corners, ids, rejectedImgPoints) =  aruco.detectMarkers(current_frame, aruco_dict, parameters=arucoParameters)

	x_axis = (corners[0][0][1][0]-corners[0][0][0][0])
	y_axis = (corners[0][0][0][1]-corners[0][0][1][1])
	angle =  atan(y_axis/x_axis)
	if x_axis<0:
		if y_axis >0:
			angle += math.pi
		elif y_axis  <0:
			angle -= math.pi
	aruco_msg.x =(corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])*0.25
	aruco_msg.y =(corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1])*0.25
	aruco_msg.theta = angle.real
	image = aruco.drawDetectedMarkers(current_frame,corners,ids)     
	cv2.imshow('Display', image)
	cv2.waitKey(1)
	#1
	
	
#gets inverse matrix 
def inverse_kinematics(Count,motion_msg,Limit):
	
	Error = Pose2D() #error from desired point
	Error.x = x_goals[Count]-aruco_msg.x
	Error.y = y_goals[Count]-aruco_msg.y
	Error.theta = theta_goals[Count]-aruco_msg.theta
	
	if abs(Error.theta)<0.05  and abs(Error.x)<3 and abs(Error.y)<3:
			
			motion_msg.x= 0
			motion_msg.y= 0
			motion_msg.z= 0
			output(motion_msg)
			rospy.sleep(1)
			print(Count)
			if Count< (Limit-1):
				Count+=1
	
	else:
	
		x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
		y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))

		motion_msg.x= (2*x/3-Error.theta*5)
		motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*5)
		motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*5)
		output(motion_msg)
		
	return motion_msg

#serialize the array
def output(motion_msg):
	
	global counter
	array = np.array([motion_msg.x,motion_msg.y,motion_msg.z], dtype=np.float32)
	serialized_array = array.tobytes()
	data = conn.recv(1024)
	print(motion_msg)
	conn.send(serialized_array)
	counter += 1
	sleep(0.3)
	#need to modify the condtion
	if counter == 10:
		s.close()


def main():
	#node name
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('/usb_cam/image_raw', Image, callback)  #sub to camera module
	print(f"Connected by {addr}")  #checking the connection

	Motion_msg = Point() #speed vector
	Rate = rospy.Rate(200) #to run the loop specific number of times
	Count = 0  #counts goal postion
	Limit = len(x_goals) #to know number of condtions
	
	#checks if points are loaded
	while Limit < 5:
		Limit = len(x_goals)
	
	#controller loop
	while not rospy.is_shutdown():
		#takes care of kinematics stuff
		Motion_msg = inverse_kinematics(Count,Motion_msg,Limit)
		
		Rate.sleep()
	rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass