import time
import sys
import navio2.util
import navio2.ublox
import navio2.pwm
import navio2.leds
import navio2.mpu9250
import navio2.lsm9ds1
from numpy import pi
import numpy as np
import struct
from math import *
from bluepy import btle
import spidev
import socket
from Servo_laulo import servo  								#servo(piece_pin,angle)
# from LED import led										#led(color)
from GPS import gps											#gps(ubl)
from GPS import init_gps								#init_gps() renvoie ubl
from ADC import voltage 									#voltage(adc,results)
from ADC import init_adc								#renvoie adc et results 
from AccelGyroMag import acc_gyr_mag,rpy		#imu(sensor='lsm')
from AccelGyroMag import init_imu_lsm					#renvoie imu comme lsm
from AccelGyroMag import init_imu_mpu					#renvoie imu comme mpu
import Calypso
from Calypso import init_Calypso,wind
from vpython import *
import os

from threading import Thread

from signal import signal, SIGPIPE, SIG_DFL 
signal(SIGPIPE, SIG_DFL) 

PORT = 65432
HOST = "navio.local"

class boat_state() :
	def __init__(self,x,y,speed,s_heading,roll,pitch,yaw,x_wind,y_wind,wind_dir) :
		self.x=x
		self.y=y
		self.speed=speed
		self.s_heading=s_heading
		self.roll=roll
		self.pitch=pitch
		self.yaw=yaw
		self.x_wind=x_wind
		self.y_wind=y_wind
		self.wind_dir=wind_dir
		self.end=False
		self.tab_rpy=np.array([[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]])

def th_gps(ubl) :
	while not b_state.end :
		gpss=gps(ubl)
		if gpss!=None :
			if gpss[0]=="lon_lat" :
				b_state.x=gpss[1]
				b_state.y=gpss[2]
			if gpss[0]=="speed" :
				b_state.speed,b_state.s_heading=gpss[0],gpss[1]*180/np.pi


def th_RPY(imu) :
	i=0
	while not b_state.end :
		roll,pitch,yaw=rpy(imu)
		b_state.tab_rpy[:,i%10]=roll,pitch,yaw
		b_state.roll=np.mean(b_state.tab_rpy[0])
		b_state.pitch=np.mean(b_state.tab_rpy[1])
		b_state.yaw=np.mean(b_state.tab_rpy[2])
		i+=1

def th_wind(dev) :
	while not b_state.end :
		vent=wind(dev)
		print(vent)
		if vent[0]=="marche" :
			b_state.x_wind,b_state.y_wind,b_state.wind_dir=vent[1:]
			b_state.wind_dir*=180/np.pi


def regu_sailboat(a,b,q=1) :
    # m=np.array([[0],[0]])
    m=np.array([[b_state.x],[b_state.y]])
    # speed=b_state.speed
    speed=0
    theta=b_state.yaw*pi/180
    a_awx=200; a_awy=2;
    # a_awx=b_state.x_wind; a_awy=b_state.y_wind;
    w_ap=np.array([[a_awx],[a_awy]])
    psi_ap = np.arctan2(w_ap[1,0],w_ap[0,0])
    a_ap=np.linalg.norm(w_ap)
    w_tr=np.array([[-speed*np.cos((theta+pi)%2*pi)+a_ap*np.cos(psi_ap+theta)],[-speed*np.sin((theta+pi)%2*pi)+a_ap*np.sin(psi_ap+theta)]])
    psi_tr=np.arctan2(w_tr[1,0],w_tr[0,0])
    a_tr=np.linalg.norm(w_tr)    
    r=0.1
    biz=pi/4
    gamma_inf=pi/4
    delta_r_max=pi/4
    e=np.linalg.det(np.hstack(((b-a)/np.linalg.norm(b-a),m-a)))
    if abs(e)>r/2 :
        q=np.sign(e)
    phi=np.arctan2((b-a)[1,0],(b-a)[0,0])
    theta_b=phi-(2*gamma_inf*np.arctan2(e,r))/pi
    if np.cos(psi_tr-theta_b)+np.cos(biz)<0 or (abs(e)<r and np.cos(psi_tr-phi)+np.cos(biz)<0) :
        theta_b=pi+psi_tr-q*biz
    if np.cos(theta-theta_b) >= 0 :
        delta_r=delta_r_max*np.sin(theta-theta_b)
    else :
        delta_r=delta_r_max*np.sign(np.sin(theta-theta_b))
    delta_s_max=(pi/2)*(np.cos(psi_tr-theta_b)+1)/2
    # print(delta_s_max)
    sigma = np.cos(psi_ap) + np.cos(delta_s_max)
    # print(sigma)
    if sigma < 0 :
        delta_s = pi + psi_ap
    else :
        delta_s = -np.sign(np.sin(psi_ap))*delta_s_max
    print(delta_s)
    return delta_r,q,delta_s


if __name__ == "__main__" :

	#Initialisation :
	#Sensors :
	navio2.util.check_apm()
	ubl=init_gps()
	adc,results=init_adc()
	lsm,mpu=init_imu_lsm(),init_imu_mpu()
	dev=init_Calypso()

	b_state=boat_state(0,0,0,0,0,0,0,0,0,0)

	#Threads :
	threads=[]
	thread_gps=Thread(None,th_gps,args=(ubl,))
	thread_RPY=Thread(None,th_RPY,args=(lsm,))
	thread_wind=Thread(None,th_wind,args=(dev,))
	threads.append(thread_wind)
	threads.append(thread_RPY)
	threads.append(thread_gps)
	for t in threads :
		t.start()

	# while not b_state.end :
	# 	x=[b_state.x,b_state.y,b_state.vx,b_state.vy,b_state.vz,]
	q=1
	while not b_state.end :
	#Simulation :
		# print("Vent x : {}, Vent y : {}, Angle : {}".format(b_state.x_wind,b_state.y_wind,b_state.wind_dir))
		# time.sleep(0.5)
		# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		# 	s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		# 	s.bind((HOST, PORT))
		# 	s.listen()
		# 	conn, addr = s.accept()
		# 	a=np.array([[-3],[4]])
		# 	b=np.array([[5],[4]])
		# 	with conn:
		# 		while True:
		# 			delta_r,q,delta_s=regu_sailboat(a,b,q)
		# 			# servo("Rudder",delta_r)
		# 			# servo("Sail",delta_s)
		# 			msg=[b_state.x,b_state.y,b_state.speed,b_state.yaw,1,0,delta_s,delta_r]
		# 			data=struct.pack('8f',*msg)
		# 			conn.sendall(data)
	#Logs saving :
		log_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log.txt')
		log_f=open(log_file,"w")
		delta_r,q,delta_s=regu_sailboat(a,b,q)
		#lon,lat,speed,s_heading,yaw,x_wind,y_wind,wind_dir,
		log_f.write("{} | {} | {} | {} | {} | {} | {} | {} | {} | {}".format(b_state.x,b_state.y,b_state.speed,b_state.s_heading,b_state.yaw,b_state.x_wind,b_state.y_wind,b_state.wind_dir,delta_r,delta_s))
		time.sleep(0.2)