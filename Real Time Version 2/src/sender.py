#!/usr/bin/env python3

import rospy
import numpy as np
import socket
from time import sleep
import signal		
import sys		
from geometry_msgs.msg import Point

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

ip = "192.168.150.171"     #Enter IP address of laptop after connecting it to WIFI hotspot


#We will be sending a simple counter which counts from 1 to 10 and then closes the socket

counter = 1
speed = Point()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((ip, 8002))
s.listen()
conn, addr = s.accept()

def output(msg):
    speed = msg
    array = np.array([0,1,2], dtype=np.float32)
    serialized_array = array.tobytes()
    loop(serialized_array)
    print("works")

def loop(serialized_array):
    data = conn.recv(1024)

    conn.send(serialized_array)
 
    counter += 1
    sleep(1)
    print(counter)
    if counter == 10:
       s.close()
   

def main():
    rospy.init_node('sender_node')
    rospy.Subscriber('sending_sig',Point,output)
    print(f"Connected by {addr}")
        
    rospy.spin()
   


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
