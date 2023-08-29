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
pi = math.pi
aruco_msg     = Pose2D()
marker_coords = np.array([[0, 0], [500, 0], [500, 500], [0, 500]], dtype=np.float32)

#goal points
x_goals     = [250,350,150,150,350]
y_goals     =   [250,300,300,150,150]
theta_goals = [0,pi/4,3*pi/4,-3*pi/4,-pi/4]

#ip address 
ip = "192.168.178.171"  

#socket lines
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((ip, 8002))
s.listen()
conn, addr = s.accept()

# to send speed
speed   = Point()
Max_speed = 150
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
	get_frame 		= br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame 	= cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	aruco_dict		= aruco.Dictionary_get(aruco.DICT_4X4_50)
	arucoParameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(current_frame , aruco_dict, parameters=arucoParameters)
	# print(ids)
	if ids is not None:
	
		new_frame = crop(corners,ids,current_frame)
		corners, ids, _ = aruco.detectMarkers(new_frame, aruco_dict, parameters=arucoParameters)
	
	else:
		print("Aruco markers not detected in image.")
	aruco_msg = pose(corners,ids)
	length    = f"{aruco_msg.x}, {aruco_msg.y}"
	cv2.putText(new_frame,length, (20,40), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
	length    = f"{aruco_msg.theta}"
	cv2.putText(new_frame,length, (20,60), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
	image     = aruco.drawDetectedMarkers(new_frame,corners,ids)  
	# print(ids)
	cv2.imshow('Display',image)
	cv2.waitKey(1)

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

def pose(corners,ids):
	global aruco_msg

	bot = corners[np.where(ids == 15)[0][0]][0]
	x_axis = (bot[1][0]-bot[0][0])
	y_axis = (bot[0][1]-bot[1][1])
	angle = atan(y_axis/x_axis)
	if x_axis<0:
		if y_axis >0:
			angle += math.pi
		elif y_axis  <0:
			angle -= math.pi
	aruco_msg.x =(bot[0][0]+bot[1][0]+bot[2][0]+bot[3][0])*0.25
	aruco_msg.y =(bot[0][1]+bot[1][1]+bot[2][1]+bot[3][1])*0.25
	aruco_msg.theta = angle.real
	# print(aruco_msg)


	return aruco_msg



#gets inverse matrix 
# def inverse_kinematics(Count,motion_msg,Limit):
	
# 	Error       = Pose2D() #error from desired point
# 	Error.x     = x_goals[Count]-aruco_msg.x
# 	Error.y     = -y_goals[Count]+aruco_msg.y
# 	Error.theta = theta_goals[Count]-aruco_msg.theta
# 	# print(x_goals[Count])
# 	# print(Count)
# 	# Error.x = 100
# 	# Error.y = 0
# 	# Error.theta = 0


# 	if abs(Error.theta)<0.05 and abs(Error.x)<3 and abs(Error.y)<3:

# 			motion_msg.x= 0
# 			motion_msg.y= 0
# 			motion_msg.z= 0
# 			output(motion_msg)
# 			# rospy.sleep(1)
# 			print(Count)
# 			if Count< (Limit-1):
# 				Count+=1
	   
# 	else:
	
# 		x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
# 		y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))
	
# 		motion_msg.x= (2*x/3-Error.theta*7)*7
# 		motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*7)*7
# 		motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*7)*7
# 		high = max(abs(motion_msg.x),abs(motion_msg.y),abs(motion_msg.z))
# 		if (Max_speed < high):
# 			ratio = high/Max_speed
# 			motion_msg.x /=ratio
# 			motion_msg.y /=ratio
# 			motion_msg.z /=ratio
# 			print(motion_msg)

# 		print(motion_msg)
# 		output(motion_msg)
		
# 	return motion_msg

# #serialize the array
def output(motion_msg):
	
	global counter
	array = np.array([motion_msg.x,motion_msg.y,motion_msg.z,0], dtype=np.short)
	serialized_array = array.tobytes()
	data = conn.recv(1024)
	# print(motion_msg)
	conn.send(serialized_array)
	counter += 1
	sleep(0.2)
	#need to modify the condtion
	if counter == 10:
		s.close()


def main():
	#node name
	global Max_speed
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('/usb_cam/image_rect', Image, callback)  #sub to camera module
	print(f"Connected by {addr}")  #checking the connection

	motion_msg = Point() #speed vector
	Rate = rospy.Rate(200) #to run the loop specific number of times
	Count = 0  #counts goal postion
	limit = len(x_goals) #to know number of condtions
	# while limit !=5:

	# 	limit = len(x_goals)
	#checks if points are loaded	
	#controller loop
	while not rospy.is_shutdown():
		#takes care of kinematics stuff
			
		Error       = Pose2D() #error from desired point
		Error.x     = x_goals[Count]-aruco_msg.x
		Error.y     = -y_goals[Count]+aruco_msg.y
		Error.theta = theta_goals[Count]-aruco_msg.theta
		# Error.x = 0
		# Error.y = 100
		# Error.theta = 0
		# aruco_msg.theta =0


		if abs(Error.theta)<0.05 and abs(Error.x)<3 and abs(Error.y)<3:

			motion_msg.x= 0
			motion_msg.y= 0
			motion_msg.z= 0
			output(motion_msg)
			sleep(1)
			print(Count)
			if Count< (limit-1):
				Count+=1
	   
		else:
	
			x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
			y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))
	
			motion_msg.x= (2*x/3-Error.theta*10)*7
			motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*10)*7
			motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*10)*7
			high = max(abs(motion_msg.x),abs(motion_msg.y),abs(motion_msg.z))
			# print(high)
			if (Max_speed < high):
				ratio = high/Max_speed
				motion_msg.x /=ratio
				motion_msg.y /=ratio
				motion_msg.z /=ratio
				
				# print(motion_msg)
			output(motion_msg)
		print(motion_msg)
		
		
		Rate.sleep()
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass