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
import math
from bluepy import btle
import spidev

from Servo_laulo import servo  								#servo(piece_pin,angle)
from LED import led										#led(color)
from GPS import gps										#gps(ubl)
from GPS import init_gps								#init_gps() renvoie ubl
from ADC import voltage 								#voltage(adc,results)
from ADC import init_adc								#renvoie adc et results 
from AccelGyroMag import acc_gyr_mag,roll,pitch,yaw		#imu(sensor='lsm')
from AccelGyroMag import init_imu_lsm					#renvoie imu comme lsm
from AccelGyroMag import init_imu_mpu					#renvoie imu comme mpu
from AccelGyroMag import vit
import Calypso
from Calypso import init_Calypso,wind


from threading import Thread

class boat_state() :
	def __init__(self,x,y,vx,vy,vz,roll,pitch,yaw,x_wind,y_wind) :
		self.x=x
		self.y=y
		self.vx=vx
		self.vy=vy
		self.vz=vz
		self.roll=roll
		self.pitch=pitch
		self.yaw=yaw
		self.x_wind=x_wind
		self.y_wind=y_wind
		self.end=False


def th_gps(ubl) :
	while not b_state.end :
		gpss=gps(ubl)
		if gpss != None :
			b_state.x,b_state.y=gpss[0],gpss[1]

def th_RPY(imu) :
	while not b_state.end :
		b_state.roll=roll(imu)
		b_state.pitch=pitch(imu)
		b_state.yaw=yaw(imu)

def th_vit(imu) :
	while not b_state.end :
		t0=time.time()

		acc,gyro,mag=acc_gyr_mag(imu)

		dt=time.time()-t0

		b_state.vx=b_state.vx+acc[0]*dt
		b_state.vy=b_state.vy+acc[1]*dt
		b_state.vz=b_state.vz+acc[2]*dt

def th_wind(dev) :
	while not b_state.end :
		b_state.x_wind,b_state.y_wind=wind(dev)[0],wind(dev)[1]

if __name__ == "__main__" :

	#Initialisation :
	#Sensors :
	navio2.util.check_apm()
	ubl=init_gps()
	adc,results=init_adc()
	lsm,mpu=init_imu_lsm(),init_imu_mpu()
#	dev=init_Calypso()

	b_state=boat_state(0,0,0,0,0,0,0,0)

	#Threads :
	threads=[]
	thread_gps=Thread(None,th_gps,args=(ubl,))
	thread_RPY=Thread(None,th_RPY,args=(lsm,))
#	thread_wind=Thread(None,th_wind,args=(dev,))
	thread_vit=Thread(None,th_vit,args=(mpu,))
#	threads.append(thread_wind)
	threads.append(thread_RPY)
	threads.append(thread_gps)
	threads.append(thread_vit)
	for t in threads :
		t.start()

	rasp=box(pos=vector(0,0,0),length=4,height=1,width=2,color=color.green)
	#Données :
	while not b_state.end :
		print("x : ", b_state.x)
		print("y : ", b_state.y)
		# rasp.pos=vector(b_state.x,b_state.y,0)
		rasp.axis=vector(b_state.roll,b_state.pitch,b_state.yaw)
		# print("vx :", b_state.vx)
		# print("vy :", b_state.vy)
		# print("vz :", b_state.vz)
		# print("roll :",b_state.roll)
		# print("pitch :",b_state.pitch)
		# print("yaw : ", b_state.yaw)
		# print("x_wind : ",b_state.x_wind)
		# print("y_wind : ", b_state.y_wind)
		# print("____________________")
		# time.sleep(0.5)