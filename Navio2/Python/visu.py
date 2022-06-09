from vpython import *
import socket
import struct
HOST = "navio.local"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

if __name__ == "__main__" :
	# scene=canvas(width=200,height=200,title="Scene",caption='Yo')
	# # earth=box(pos=vector(0,-1,0),length=1000,height=1,width=1000,color=color.blue)
	# rasp=box(pos=vector(0,0,0),axis=vector(0,0,0),length=4,height=1,width=2,color=color.red)
	# # arr_b=pyramid(pos=rasp.pos+vector(2,0,0),color=color.red,length=1,height=1,width=2)

	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	    s.connect((HOST, PORT))
	    while True :
		    data = s.recv(1024)
		    loc=struct.unpack('2f',data)
		    print(loc)