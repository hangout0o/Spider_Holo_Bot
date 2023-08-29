import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2
import cv2.aruco as aruco				# OpenCV Library
from cmath import atan # If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation
from geometry_msgs.msg import Point 
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
import numpy as np
import socket
import sys
import signal		# To handle Signals by OS/user
from time import sleep
from cv_basics.msg import aruco_data

info = aruco_data()
mes = String()
def cont(msg):
    global mes
    mes = msg

def aruco_msg(msg):
    global info
    # print(f'aruco_msg; {msg}')
    info = msg

def main():
#node name
    rospy.init_node('checker_node')
    rospy.Subscriber('/contours',String, cont)
    rospy.Subscriber('/detected_aruco',aruco_data, aruco_msg)
    while(True):
        print(info)
        print(mes)

    rospy.spin()

if __name__ == "__main__":
    
	try:
		main()
	except rospy.ROSInterruptException:
		pass