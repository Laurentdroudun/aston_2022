from vpython import *
import socket
import struct
import time
from math import *
HOST = "navio.local"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

if __name__ == "__main__" :
	rasp=box(pos=vector(0,0,0),length=4,height=1,width=2,color=color.red)
	mast=cylinder(pos=vector(0,0.5,0),length=3,axis=vector(0,1,0),radius=0.2,color=color.red)
	front_b=pyramid(pos=vector(2,0,0),axis=vector(1,0,0),size=vector(2,1,2),color=color.red)
	rudder=box(pos=vector(-2,-0.5,0),axis=vector(1,0,0),size=vector(1,1.2,0.5))
	sail=triangle(v0=vertex(pos=vec(-0.2,3.5,0)),v1=vertex(pos=vec(-0.2,1,0)),v2=vertex(pos=vec(-2,1,0)))
	mast_sail=compound([mast,sail])
	s_boat=compound([rasp,mast_sail,front_b,rudder])
	x_axis=arrow(pos=vector(0,0,0),axis=vector(1,0,0),color=color.blue)
	y_axis=arrow(pos=vector(0,0,0),axis=vector(0,0,1),color=color.green)
	z_axis=arrow(pos=vector(0,0,0),axis=vector(0,-1,0),color=color.orange)
	x_b=arrow(pos=vector(0,0,0),axis=vector(s_boat.axis.x,0,0))
	# y_b=arrow(pos=s_boat.pos,axis=vector(0,s_boat.axis.y,0))
	# z_b=arrow(pos=s_boat.pos,axis=vector(0,0,s_boat.axis.z))
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
			if abs(old_z-z) < 100 :
				x_b.axis=s_boat.axis
				# y_b.axis=vector(0,s_boat.axis.y,0)
				# z_b.axis=vector(0,0,s_boat.axis.z)
				s_boat.rotate(angle=-radians(old_x-x),axis=s_boat.axis,origin=vector(0,0,0))
				# s_boat.rotate(angle=-radians(old_y-y),axis=vector(-rasp.axis.z,0,rasp.axis.z),origin=vector(0,0,0))
				s_boat.rotate(angle=radians(old_z-z),axis=vector(0,-1,0),origin=vector(0,0,0))
				print(radians(old_z-z))
			old_x,old_y,old_z=x,y,z
			time.sleep(0.1)