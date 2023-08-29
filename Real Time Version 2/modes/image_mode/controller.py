#!/usr/bin/env python3

######################## IMPORT MODULES ##########################
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2  # OpenCV Library
import cv2.aruco as aruco #for detection of aruco marker
from cmath import atan # for getting angle of aruco marker
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation
from geometry_msgs.msg import Point  # used for motion msg for 3 wheels as x,y,z
from std_msgs.msg import String #for pub the final list of controus
from std_msgs.msg import Int32 #for pub few datas
import math # for pi accuracy
import numpy as np #for serialize the int array
import socket # sending data through wifi
import sys 
import signal		# To handle Signals by OS/user
from time import sleep #to some small delay for con send
from cv_basics.msg import aruco_data # as per submission need
############################ GLOBALS #############################

pi = math.pi # for pi detecion
marker_coords = np.array([[0, 0], [500, 0], [500, 500], [0, 500]], dtype=np.float32) # co-ordinates of cropped window

#for submission need
aruco_publisher = rospy.Publisher('/detected_aruco',aruco_data, queue_size=10) 
contourPub = rospy.Publisher('/contours',String, queue_size=10)
taskStatusPub = rospy.Publisher('/taskStatus',Int32, queue_size=10)
penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)


#goal points
xList , yList,thetaList , xListFinal , yListFinal ,thetaFinal= [] , [] , [] , [],[],[]
aruco_msg = aruco_data()  # stores the current location of the bot
cData = String() #for pub the final list of controus 
penData = Int32() # pen status
Max_speed = 250 # max speed to avoid torque issues and overshot
motion_msg = Point() #speed vector
Kp = 17 # constant for p controller

#ip address 
ip = "192.168.173.171" #ip address of my device (dont hack me)

#socket lines
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((ip, 8002))
s.listen()
conn, addr = s.accept()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

## cleanup and signal_handler are clear up fucntion when program ends
def signal_handler(sig, frame):
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	motion_msg.x= 0
	motion_msg.y= 0
	motion_msg.z= 0
	penData.data= 0
	output(motion_msg)
	s.close()
	print("cleanup done")

def function_mode():
	xList.clear()
	yList.clear()
	thetaList.clear()

	for sample in range(0,314*2*2):
		#taking 200 sample for curve
		t = sample/200
		#iterating for appending each points
		xList.append(200*math.cos(t)+249.5)
		yList.append(100*math.sin(2*t)+249.5)
		thetaList.append(pi/2-pi/4*math.sin(t))
	#storing in final list
	xListFinal.append(xList.copy())
	yListFinal.append(yList.copy())
	thetaFinal.append(thetaList.copy())

def image_mode():
	global cData,xListFinal,yListFinal,xList,yList
	# read the image and we resized it and found controus and choose particular contorus of our wish for drawing
	img =  cv2.imread("/home/asslam/Downloads/taskImages/snapchat.png")
	new0 = cv2.resize(img, (int(img.shape[1]*0.8),int(img.shape[0]*0.8)), interpolation = cv2.INTER_LINEAR)
	new = cv2.resize(new0, (500, 500), interpolation = cv2.INTER_LINEAR)
	gray= cv2.cvtColor(new,cv2.COLOR_BGR2GRAY)
	_ , thresh = cv2.threshold(gray,50,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	contours = [contours[i]for i in [2]]
	#adding them in the list
	for i in contours: 
		xList.clear()    #clearing any previous data from the lists
		yList.clear()
		thetaList.clear()
	#iterating through the nested list
		for j in i:
	#temporary appending the x,y coordinates in separate lists
			xList.append(int(j[0][0]))
			yList.append(int(j[0][1]))
			thetaList.append(0)
		# print(xList)
		xListFinal.append(xList.copy())
		yListFinal.append(yList.copy())
		thetaFinal.append(thetaList.copy())
	#pub them for submission needs
	cData.data = str([xListFinal,yListFinal])
	contourPub.publish(cData)


# gets the co-ordinates of the bot
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
		#cropping the new frame based on 4 corner aruco markers
		new_frame = crop(corners,ids,current_frame)
		#again detecting markers for new frame to find the co-orindate of aruco marker
		corners, ids, _ = aruco.detectMarkers(new_frame, aruco_dict, parameters=arucoParameters)
		# if ids is not None:
		pose(corners,ids) # getting the current positon of aruco marker
		length    = f"{aruco_msg.x}, {aruco_msg.y}"
		cv2.putText(new_frame,length, (20,40), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
		length    = f"{aruco_msg.theta}"
		cv2.putText(new_frame,length, (20,60), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
		image     = aruco.drawDetectedMarkers(new_frame,corners,ids)  
		cv2.imshow('Display',image)
		cv2.waitKey(1)
	else:
		print("Aruco markers not detected in image")

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
	
	global aruco_msg,aruco_publisher
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
	# aruco_publisher.publish(aruco_msg)

#gets inverse matrix 
def inverse_kinematics(Error,motion_msg):
	
	x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
	y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))

	motion_msg.x= (2*x/3-Error.theta*10)*Kp
	motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*10)*Kp
	motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*10)*Kp

	high = max(abs(motion_msg.x),abs(motion_msg.y),abs(motion_msg.z))
	if (Max_speed < high):
		ratio = high/Max_speed
		motion_msg.x /=ratio
		motion_msg.y /=ratio
		motion_msg.z /=ratio

	return motion_msg

#serialize the array
def output(motion_msg):

	array = np.array([motion_msg.x,motion_msg.y,motion_msg.z,penData.data], dtype=np.short)
	serialized_array = array.tobytes()
	data = conn.recv(1024)
	conn.send(serialized_array)
	#any way i can reduce further
	sleep(0.2)


def main():
	#node name
	rospy.init_node('controller_node')
	signal.signal(signal.SIGINT, signal_handler)

	function_mode()
	
	taskStatus = Int32()
	taskStatus.data = 0
	penData.data = 0
	cData.data = str([xListFinal,yListFinal])
	taskStatusPub.publish(taskStatus)
	penPub.publish(penData)
	contourPub.publish(cData)

	rospy.Subscriber('/usb_cam/image_rect', Image, callback)
	#rospy.Subscriber('endSignal',int,endSignalCb)
	print(f"Connected by {addr}")  #checking the connection

	Rate = rospy.Rate(200) #to run the loop specific number of times
	#check on this
	total_count = 0  #counts goal postion
	ind_count = 0

	limit = len(xListFinal)
	length = []

	for i in range(len(xListFinal)):
		length.append(len(xListFinal[i])) 
	

	#controller loop
	while not rospy.is_shutdown():
		#takes care of kinematics stuff

		Error       = Pose2D() #error from desired point
		Error.x     =  xListFinal[total_count][ind_count]-aruco_msg.x
		Error.y     = -yListFinal[total_count][ind_count]+aruco_msg.y
		Error.theta =  thetaFinal[total_count][ind_count]-aruco_msg.theta
		#as of now sunday moringing 0.1,5,5
		if abs(Error.theta)<0.1 and abs(Error.x)<5 and abs(Error.y)<5:
			#first time pen down
			if ind_count < (length[total_count]-1):
				ind_count +=1
				penData.data=1
			elif total_count < (limit-1):
				penData.data= 0
				total_count +=1
				ind_count = 0
			else:
				# motion_msg.x= 0
				# motion_msg.y= 0
				# motion_msg.z= 0
				# penData.data= 0
				# output(motion_msg)
				cleanup()
				taskStatus.data = 1
				penPub.publish(penData)
				taskStatusPub.publish(taskStatus)
				sys.exit(0)
			cData.data = str([xListFinal,yListFinal])
			penPub.publish(penData)
		else:
			inverse_kinematics(Error,motion_msg)
			output(motion_msg)
		aruco_publisher.publish(aruco_msg)
		#need to change in future
		Rate.sleep()
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass