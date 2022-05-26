import spidev
import time 
import argparse 
import sys 
import navio2.mpu9250 
import navio2.util 
from math import * 
from numpy import *


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

	print ("Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]),)
	print (" Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]),)
	print (" Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2]))
	yaw=atan2(m9m[0],m9m[1])*180/pi

def yaw(imu) :
	m9a, m9g, m9m = imu.getMotion9()
	yaw=atan2(m9m[0],m9m[1])*180/pi
	return("YAW :",yaw)


if __name__ == "__main__" :
    imu=init_imu_lsm()
    tab_mag=[]
    tab_accz=[]
    tab_accy=[]
    tab_accx=[]
    dt=0.5
    while True :
        acc_gyr_mag(imu)
        print("IMU Calibration")
        print("________________")
        """print("Magnetometer Calibration Starting")
        time.sleep(5)
        print("Getting data")
        for t in arange(0,180,1) :
            m9a,m9g,m9m=imu.getMotion9()
            tab_mag.append(m9m)
            time.sleep(dt)
            if t%60==0 :
                print("30s")
        tab_mag=array(tab_mag)
        print("Data collected")
        offset_x=mean(tab_mag[:,0])
        offset_y=mean(tab_mag[:,1])
        offset_z=mean(tab_mag[:,2])
        print("Offsets :",offset_x,",",offset_y,",",offset_z)
        print("End of Accelerometer Calibration")
        print("________________")"""
        print("Accelerometer Calibration Starting")
        print("Getting data")
        print("Put the card on a flat surface")
        print("Starting in 5 seconds")
        time.sleep(5)
        print("Starting")
        for t in arange(0,60,1) :
            m9a,m9g,m9m=imu.getMotion9()
            tab_accz.append(m9a)
            time.sleep(0.5)
        print("Rotate the card around x-axis")
        print("Starting in 5 seconds")
        time.sleep(5)
        print("Starting")
        for t in arange(0,60,1) :
             m9a,m9g,m9m=imu.getMotion9()
             tab_accy.append(m9a)
             time.sleep(0.5)
        print("Rotate the card around z-axis")
        print("Starting in 5 seconds") 
        time.sleep(5)
        print("Starting")
        for t in arange(0,60,1) :
             m9a=imu.getMotion9()[0]
             tab_accx.append(m9a)
             time.sleep(0.5)
        print("Data collecting ended")
        tab_accz=array(tab_accz)
        tab_accy=array(tab_accy)
        tab_accx=array(tab_accx)
        print(tab_accz,"\n",tab_accy,"\n",tab_accx,"\n")
        mean_x=mean([mean(tab_accz[:,0]),mean(tab_accy[:,0]),mean(tab_accx[:,0]-9.80665)])
        mean_y=mean([mean(tab_accz[:,1]),mean(tab_accy[:,1]-9.80665),mean(tab_accx[:,1])])
        mean_z=mean([mean(tab_accz[:,2]-9.80665),mean(tab_accy[:,2]),mean(tab_accx[:,2])])
        print("Offset :",mean_x,",",mean_y,",",mean_z)
        print("End")
