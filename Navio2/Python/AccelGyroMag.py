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
	# imu.read_all()
	# imu.read_gyro()
	# imu.read_acc()
	# imu.read_temp()
	# imu.read_mag()
	m9a, m9g, m9m = imu.getMotion9()
	offset_acc_x=-0.00048706361666657944		#Offsets found thanks to calibration
	offset_acc_y=-0.1341660862033335
	offset_acc_z=-0.2977952716666666
	m9a[0]-=offset_acc_x
	m9a[1]-=offset_acc_y
	m9a[2]-=offset_acc_z
	# print ("Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]),)
	# print (" Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]),)
	# print (" Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2]))
	return m9a,m9g,m9m


def yaw(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)

	yaw=atan2(m9m[1],m9m[0])*180/pi
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
	# mag_x=m9m[0]*cos(pitch)+m9m[1]*sin(roll)*sin(pitch)+m9m[2]*cos(roll)*sin(pitch)
	# mag_y=m9m[1]*cos(roll)-m9m[2]*sin(roll)
	# yaw=180*atan2(mag_y,mag_x)/pi
	yaw=atan2(m9m[1],m9m[0])*180/pi
	return [roll,pitch,yaw]

if __name__=="__main__" :
	while True :
		imu=init_imu_lsm()
		roll,pitch,yaw=rpy(imu)
		print("Roll :",roll)
		print("Pitch :", pitch)
		print('Yaw :',yaw)
		time.sleep(0.5)