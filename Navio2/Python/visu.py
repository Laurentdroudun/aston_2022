from vpython import *
import socket
import struct
from math import *
HOST = "navio.local"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

if __name__ == "__main__" :
	# scene=canvas(width=200,height=200,title="Scene",caption='Yo')
	# # earth=box(pos=vector(0,-1,0),length=1000,height=1,width=1000,color=color.blue)
	rasp=box(pos=vector(0,0,0),length=4,height=1,width=2,color=color.red)
	mast=cylinder(pos=vector(0,0.5,0),length=3,axis=vector(0,1,0),radius=0.2,color=color.red)
	front_b=pyramid(pos=vector(2,0,0),axis=vector(1,0,0),size=vector(2,1,2),color=color.red)
	rudder=box(pos=vector(-2,-0.5,0),axis=vector(1,0,0),size=vector(1,1.2,0.5))
	sail=triangle(v0=vertex(pos=vec(-0.2,3.5,0)),v1=vertex(pos=vec(-0.2,1,0)),v2=vertex(pos=vec(-2,1,0)))
	mast_sail=compound([mast,sail])
	s_boat=compound([rasp,mast_sail,front_b,rudder])
	# arr_b=pyramid(pos=rasp.pos+vector(2,0,0),color=color.red,length=1,height=1,width=2)
	x_axis=arrow(pos=vector(0,0,0),axis=vector(1,0,0),color=color.blue)
	y_axis=arrow(pos=vector(0,0,0),axis=vector(0,0,1),color=color.green)
	z_axis=arrow(pos=vector(0,0,0),axis=vector(0,-1,0),color=color.orange)
	old_x,old_y,old_z=0,0,0
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.connect((HOST, PORT))
		while True :
			try :
				data = s.recv(1024)
				rpy=struct.unpack('3f',data)
			except :
				continue
			x,y,z=rpy
			rasp.pos=vector(0,0,0)
			s_boat.rotate(angle=-radians(old_x-x),axis=vector(cos(radians(z)),0,-sin(radians(z))),origin=vector(0,0,0))
			s_boat.rotate(angle=-radians(old_y-y),axis=vector(sin(radians(z)),0,cos(radians(z))),origin=vector(0,0,0))
			s_boat.rotate(angle=radians(old_z-z),axis=vector(0,-1,0),origin=vector(0,0,0))
			# mast.axis=vector(-rasp.axis.y,rasp.axis.z,rasp.axis.x)
			old_x,old_y,old_z=x,y,z
			print(s_boat.axis)