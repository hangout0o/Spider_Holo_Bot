import socket
from time import sleep
import signal		
import sys		
import numpy as np

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

ip ="192.168.156.171"   #Enter IP address of laptop after connecting it to WIFI hotspot


#We will be sending a simple counter which counts from 1 to 10 and then closes the socket
counter = 1

#To undeerstand the working of the code, visit https://docs.python.org/3/library/socket.html
s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((ip, 8002))
s.listen()
conn, addr = s.accept()

print(f"Connected by {addr}")

while True:
    input1 = int(input())
    if input1==0:
        size = [1,0,0,0] 
    elif  input1 ==1:  
        size = [0,1,0,0]
    elif  input1 ==2:  
        size = [0,0,1,0] 
    elif  input1 ==3:  
        size = [0,0,1,1] 
    data = conn.recv(1024)
    array = np.array(size, dtype=np.short)
    serialized_array = array.tobytes()
    data = conn.recv(1024)
    print(counter)
    # print(data)
    conn.sendall(serialized_array)
    counter += 1
    sleep(0.2)
            


