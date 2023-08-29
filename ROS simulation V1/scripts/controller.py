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

# Team ID:		[ HB#1310 ]
# Author List:		[ Vaseekaran E R,Mohamed Asslam B,Shriharshini R,Hurath R]
# Filename:		controller.py
# Functions:
#			[ main, ruco_feedback_Cb, task2_goals_Cb,signal_handler]
# Nodes:		/right_wheel_force,/front_wheel_force,/left_wheel_force --> Publisher
#               detected_aruco,task2_goals --> Subscriber

################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user
import numpy as np
from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
import cv2

from time import sleep
import math		
from cmath import pi
from cmath import sqrt	# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles
import time

################## GLOBAL VARIABLES ######################



hola_x = 249.5
hola_y = 249.5
hola_theta = 0

x_goals = []
y_goals = []
theta_goals = []
xList , yList,thetaList , xListFinal , yListFinal ,thetaFinal= [] , [] , [] , [],[],[]


right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

force =[0,0,0]
Max_speed = 250
penData = 0
kd = 0.4
kp = 0.6

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	# cleanup()
	sys.exit(0)

# def cleanup():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################


def function_mode():
	xList.clear()
	yList.clear()
	thetaList.clear()

	for sample in range(0,314*2,3):
		t = sample/100
		xList.append(200*math.cos(t)+249.5)
		yList.append(100*math.sin(2*t)+249.5)
		thetaList.append(-pi/4*math.sin(t)+pi/2)
	xListFinal.append(xList.copy())
	yListFinal.append(yList.copy())
	thetaFinal.append(thetaList.copy())

def image_mode():
	img =  cv2.imread("/home/asslam/Downloads/taskImages/smile.png")
	new   = cv2.resize(img, (500, 500), interpolation = cv2.INTER_LINEAR)
	gray= cv2.cvtColor(new,cv2.COLOR_BGR2GRAY)
	_, thresh = cv2.threshold(gray,50,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	contours = [contours[i]for i in [1,3,4,5]]

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
	#finally appending the list separated x,y coordinates in final list


def aruco_feedback_Cb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta


#def inverse_kinematics():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################


def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)
	function_mode()

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)


	vel1 = Wrench()
	vel2 = Wrench() 
	vel3 = Wrench() 
	rate = rospy.Rate(100)

	total_count = 0  #counts goal postion
	ind_count = 0
	limit = len(xListFinal)
	length = []
	for i in range(len(xListFinal)):
		length.append(len(xListFinal[i])) 
	pen_mode = 0
	print(f'limit:{limit},length:{length}')
	pre_errror = Pose2D()
	derivative = Pose2D()
	pre_errror.x = 0
	pre_errror.y =0
	pre_errror.theta =0
	final_error = Pose2D()
	# pre_time = 0
	# current_time = time.time()

	while not rospy.is_shutdown():
		Error_x = xListFinal[total_count][ind_count] - hola_x
		Error_y =-yListFinal[total_count][ind_count] + hola_y
		Error_theta = thetaFinal[total_count][ind_count] - hola_theta
		#current_time = time.time()
		# derivative.x = (Error_x - pre_errror.x)#/(current_time-pre_time)
		# derivative.y = (Error_y - pre_errror.y)#/(current_time-pre_time)
		# derivative.theta = (Error_theta - pre_errror.theta)#/(current_time-pre_time)
		
		final_error.x = kp*Error_x+kd*(Error_x - pre_errror.x)
		final_error.y = kp*Error_y+kd*(Error_y - pre_errror.y)
		final_error.theta = kp*Error_theta+kd*(Error_theta - pre_errror.theta)

		pre_errror.x = Error_x
		pre_errror.y = Error_y
		pre_errror.theta = Error_theta

		if abs(Error_theta)<0.05 and abs(Error_x)<1 and abs(Error_y)<1:

			
			if ind_count < (length[total_count]-1):
				penData = 1
				print(penData)
				ind_count +=1
			elif total_count < limit-1:
				pen_mode = 0
				print(penData)
				total_count +=1
				ind_count = 0
				print(f'penmodee:{pen_mode}')
			else:
				penData = 0
				print(penData)
				vel1.force.x = force[0]*0
				vel2.force.x= force[1]*0
				vel3.force.x =  force[2]*0
				front_wheel_pub.publish(vel1)
				left_wheel_pub.publish(vel2)
				right_wheel_pub.publish(vel3)
				print("finished")
				break

		else:
			x = (final_error.x*math.cos(hola_theta) + final_error.y*math.sin(hola_theta))
			y = (-final_error.x*math.sin(hola_theta) + final_error.y*math.cos(hola_theta))

			force[0] = 2*x/3 -final_error.theta
			force[1] = -x/3+y/math.sqrt(3)-final_error.theta
			force[2] = -x/3-y/math.sqrt(3)-final_error.theta
	
			vel1.force.x = force[0]*70
			vel2.force.x= force[1]*70
			vel3.force.x =  force[2]*70
			# vel1.force.x = force[0]*0
			# vel2.force.x= force[1]*0
			# vel3.force.x =  force[2]*0
			high = max(vel1.force.x,vel2.force.x,vel3.force.x)
			if (Max_speed < high):
				ratio = high/Max_speed
				vel1.force.x /=ratio
				vel2.force.x /=ratio
				vel3.force.x /=ratio


			front_wheel_pub.publish(vel1)
			left_wheel_pub.publish(vel2)
			right_wheel_pub.publish(vel3)
		#pre_time = current_time
		rate.sleep()

		############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

