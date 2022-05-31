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
	offset_acc=[offset_acc_x,offset_acc_y,offset_acc_z]
	m9a-+offset_acc
	# print ("Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]),)
	# print (" Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]),)
	# print (" Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2]))
	return m9a


def yaw(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	yaw=atan2(m9m[0],m9m[1])*180/pi
	return("YAW :",yaw)

def roll(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	roll=atan2(m9m[1],m9m[2])*180/pi
	return("ROLL :",roll)

def pitch(imu) :
	m9a, m9g, m9m = acc_gyr_mag(imu)
	pitch=atan2(m9m[2],m9m[0])*180/pi
	return("PITCH :",pitch)
