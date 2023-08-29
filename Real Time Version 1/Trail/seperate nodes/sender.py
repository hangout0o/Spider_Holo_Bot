#!/usr/bin/env python3
import numpy as np
import socket
from time import sleep
import signal		
import sys		


def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

ip = "192.168.17.171"     #Enter IP address of laptop after connecting it to WIFI hotspot


#We will be sending a simple counter which counts from 1 to 10 and then closes the socket

counter = 0

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((ip, 8002))
s.listen()
conn, addr = s.accept()

def output():
    
    array = np.array([0,100,-100,1], dtype=np.short)
    serialized_array = array.tobytes()
    loop(serialized_array)

def loop(serialized_array):
    global counter ,s
    
    if counter == 10:
        s.close()
    else:
        data = conn.recv(1024)
        print(counter)
            # print(data)
        conn.send(serialized_array)
            #str.encode(str(counter))
        counter += 1
        sleep(0.2)
      
   

def main():
    print(f"Connected by {addr}") 
    work = True
    while(True):
     output()
   


if __name__ == "__main__":
		main()
	
