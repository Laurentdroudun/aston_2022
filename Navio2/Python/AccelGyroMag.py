import spidev
import time
import argparse 
import sys
import navio2.mpu9250
import navio2.util
from math import *

navio2.util.check_apm()

def init_imu_lsm() :
	imu=navio2.lsm9ds1.LSM9DS1()
	imu.initialize()
	return imu

def init_imu_mpu() :
	imu=navio2.mpu9250.MPU9250()
	imu.initialize()
	return imu

def acc_gyr_mag(imu):
	m9a, m9g, m9m = imu.getMotion9()
	offset_acc=[]
	offset_mag=[]
	offset_acc.append(0.085833271023372)
	offset_acc.append(0.015160223023640923)
	offset_acc.append(0.015160223023640923)
	offset_mag.append(-7.959210618259436)
	offset_mag.append(-21.521659859117926)
	offset_mag.append(-11.650658574455132)
	for k in range(3) :
		m9a[k]-=offset_acc[k]
	print(m9a)
	for k in range(3) :
		m9a[k]/=9.81
		m9m[k]-=offset_mag[k]
	print(m9a)
	return m9a,m9g,m9m


def yaww(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	yaw=180*atan(m9a[2]/sqrt(m9a[0]**2+m9a[1]**2))/pi
	# yaw=atan2(m9m[1],m9m[0])*180/pi
	return(yaw)

def roll(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	# roll=atan2(m9m[2],m9m[1])*180/pi
	roll=180*atan2(-m9a[0],sqrt(m9a[1]**2+m9a[2]**2))/pi
	return(roll)

def pitch(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	# pitch=atan2(m9m[2],m9m[0])*180/pi
	pitch=180*atan2(m9a[1],sqrt(m9a[0]**2+m9a[2]**2))/pi
	return(pitch)

def rpy(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	roll=180*atan2(m9a[1],sqrt(m9a[0]**2+m9a[2]**2))/pi
	pitch=180*atan2(m9a[0],sqrt(m9a[1]**2+m9a[2]**2))/pi
	yaw=atan2(m9m[1],m9m[0])*180/pi
	return roll,pitch,yaw

if __name__=="__main__" :
	while True :
		imu=init_imu_lsm()
		roll,pitch,yaw=rpy(imu)
		print("Roll :",roll)
		print("Pitch :", pitch)
		print('Yaw :',yaw)
		time.sleep(0.5)