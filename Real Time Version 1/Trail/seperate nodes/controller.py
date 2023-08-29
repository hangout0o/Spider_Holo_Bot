#!/usr/bin/env python3

################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Point		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		
from cmath import pi
from cmath import sqrt	# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

hola_x = 249.5
hola_y = 249.5
hola_theta = 0

x_goals = [0,0,0,0,0,0,0,0,0,0,0,0,0]
y_goals = [0,0,0,0,0,0,0,0,0,0,0,0,0]
theta_goals = [0,0,0,0,0,0,0,0,0,0,0,0,0]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

force =[0,0,0]

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
  
  
# def task2_goals_Cb(msg):
# 	global x_goals, y_goals, theta_goals
# 	x_goals.clear()
# 	y_goals.clear()
# 	theta_goals.clear()

# 	for waypoint_pose in msg.poses:
# 		x_goals.append(waypoint_pose.position.x)
# 		y_goals.append(waypoint_pose.position.y)

# 		orientation_q = waypoint_pose.orientation
# 		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
# 		theta_goal = euler_from_quaternion (orientation_list)[2]
# 		theta_goals.append(theta_goal)

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

	motion_publisher = rospy.Publisher('sending_sig',Point)
	motion_msg = Point()
	

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	# rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)

	rate = rospy.Rate(100)
	Kp= 0.5
	limit = len(x_goals)
	Count = 0
	while limit < 5:
		limit = len(x_goals)
	
	while not rospy.is_shutdown():
		Error_x = +x_goals[Count] - hola_x
		Error_y = -y_goals[Count] + hola_y
		Error_theta = theta_goals[Count] - hola_theta

		if  abs(Error_theta)<0.05  and abs(Error_x)<5 and abs(Error_y)<5:
			Kp = 0

		x = (Error_x*math.cos(hola_theta) + Error_y*math.sin(hola_theta))
		y = (-Error_x*math.sin(hola_theta) + Error_y*math.cos(hola_theta))

		force[0] = 2*x/3-Error_theta*5
		force[1] = -x/3+y/math.sqrt(3)-Error_theta*5
		force[2] = -x/3-y/math.sqrt(3)-Error_theta*5
	
		motion_msg.x= force[0]*Kp
		motion_msg.y= force[1]*Kp
		motion_msg.z =  force[2]*Kp
		# motion_msg[0]+=1
		# motion_msg[1]+= 2
		# motion_msg[2] += 0.2
		print(f'{motion_msg.x} {motion_msg.y} {motion_msg.z}')
		motion_publisher.publish(motion_msg)

		rospy.sleep(3)
		rate.sleep()
		if Kp == 0:
			rospy.sleep(3)
			print(Count)
			if Count< (limit-1):
				Count+=1
				Kp= 4

	

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

