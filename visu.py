from vpython import *
import socket
import struct
import time
import numpy as np
from numpy import sin,cos
from math import *
import os
HOST = "navio.local"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

if __name__ == "__main__" :
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.connect((HOST, PORT))
		old_yaw,old_delta_s,old_delta_r=0,0,0
		data=s.recv(1024)
		data=struct.unpack('8f',data)
		x,y,speed,yaw,x_wind,y_wind,delta_s,delta_r=data
		scene_b=canvas(center=vector(x,0,-y))
		rasp=box(canvas=scene_b,pos=vector(0,0,0),length=4,height=1,width=2,color=color.red)
		mast=cylinder(canvas=scene_b,pos=vector(0,0.5,0),length=3,axis=vector(0,1,0),radius=0.2,color=color.red)
		front_b=pyramid(canvas=scene_b,pos=vector(2,0,0),axis=vector(1,0,0),size=vector(2,1,2),color=color.red)
		rudder=box(canvas=scene_b,pos=vector(-2,-0.5,0),axis=vector(1,0,0),size=vector(1,1.2,0.5))
		sail=triangle(canvas=scene_b,v0=vertex(pos=vec(-0.2,3.5,0)),v1=vertex(pos=vec(-0.2,1,0)),v2=vertex(pos=vec(-2,1,0)))
		mast_sail=compound([mast,sail])
		s_boat=compound([rasp,front_b])
		line_to_follow=cylinder(canvas=scene_b,pos=vector(-5,0,-3),length=15,axis=vector(1,0,0),radius=0.1,color=color.red)
		x_axis=arrow(canvas=scene_b,pos=vector(0,0,0),axis=vector(1,0,0),color=color.blue)
		wind=arrow(canvas=scene_b,pos=vector(-4,2,-2),axis=vector(1,0,0),color=color.white)
		y_axis=arrow(canvas=scene_b,pos=vector(0,0,0),axis=vector(0,0,1),color=color.green)
		z_axis=arrow(canvas=scene_b,pos=vector(0,0,0),axis=vector(0,-1,0),color=color.orange)
		i=0
		while True :
			try :
				data = s.recv(1024)
				data=struct.unpack('8f',data)
			except :
				continue
			x,y,speed,yaw,x_wind,y_wind,delta_s,delta_r=data
			
			print("x = {} | y = {} | speed = {} | yaw = {} | x_wind = {} | y_wind = {} | delta_s = {} | delta_r = {}\n\n".format(x,y,speed,yaw,x_wind,y_wind,delta_s,delta_r))
			# if abs(old_yaw-yaw) < 50 :
			s_boat.pos=vector(x,0,-y)
			s_boat.rotate(angle=radians(old_yaw-yaw),axis=vector(0,-1,0),origin=s_boat.pos)
			mast_sail.rotate(angle=radians(old_yaw-yaw)+old_delta_s-delta_s,axis=vector(0,-1,0),origin=s_boat.pos)
			rudder.rotate(angle=radians(old_yaw-yaw),axis=vector(0,-1,0),origin=s_boat.pos)
			rudder.rotate(angle=old_delta_r-delta_r,axis=vector(0,-1,0))
			print(old_delta_r-delta_r)
			old_yaw,old_delta_s,old_delta_r=yaw,delta_s,delta_r
			time.sleep(0.2)








				# Rx=np.array([[1,0,0],[0,cos(x),-sin(x)],[0,sin(x),cos(x)]])
				# Ry=np.array([[cos(y),0,sin(y)],[0,1,0],[-sin(y),0,cos(y)]])
				# Rz=np.array([[cos(yaw),-sin(yaw),0],[sin(yaw),cos(yaw),0],[0,0,1]])
				# axe_y=Rz@Ry@Rx@np.array([[1],[0],[0]])
				# print(axe_y)
				# y_b.axis=vector(axe_y[0,0],axe_y[2,0],axe_y[1,0])
				# s_boat.rotate(angle=-radians(old_x-x),axis=s_boat.axis,origin=vector(0,0,0))
				# s_boat.rotate(angle=-radians(old_y-y),axis=vector(-sin(radians(x)),0,cos(radians(x))),origin=vector(0,0,0))