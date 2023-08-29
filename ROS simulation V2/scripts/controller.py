#!/usr/bin/env python3

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import numpy
import math		
from cmath import pi
from cmath import sqrt	# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14
hola_thetaa = [pi/2,-pi/2,0,-pi]  #---- for L
# hola_thetaa = [pi/12,-7*pi/12,3*pi/4] ------ for triangle
Error_theta = 0


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


def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)


	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	
	vel1 = Wrench()
	vel2 = Wrench() 
	vel3 = Wrench() 
	rate = rospy.Rate(100)
	Kp= 4
	speed =50
	
	rospy.sleep(3)

	while not rospy.is_shutdown():
	
	    #loop
		for hola_theta in hola_thetaa:

		# for hola_theta in numpy.arange(-pi/4,7*pi/4,0.1):
			Error_x = speed
			Error_y = 0
			rospy.loginfo(hola_theta)
			x = (Error_x*math.cos(hola_theta) + Error_y*math.sin(hola_theta))
			y = (-Error_x*math.sin(hola_theta) + Error_y*math.cos(hola_theta))
			force[0] = 2*x/3-Error_theta*5
			force[1] = -x/3+y/math.sqrt(3)-Error_theta*5
			force[2] = -x/3-y/math.sqrt(3)-Error_theta*5
			vel1.force.x = force[0]*Kp
			vel2.force.x= force[1]*Kp
			vel3.force.x =  force[2]*Kp
			
			front_wheel_pub.publish(vel1)
			left_wheel_pub.publish(vel2)
			right_wheel_pub.publish(vel3)
			rate.sleep()
			rospy.sleep(3)
		
    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

