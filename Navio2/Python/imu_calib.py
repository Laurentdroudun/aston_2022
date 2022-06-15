import spidev
import time 
import argparse 
import sys 
import navio2.mpu9250
import navio2.lsm9ds1
import navio2.util 
from math import * 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize
import os

navio2.util.check_apm()

def init_lsm():
	imu=navio2.lsm9ds1.LSM9DS1()
	imu.initialize()
	return imu

def init_mpu() :
	imu=navio2.mpu9250.MPU9250()
	imu.initialize()
	return imu

def acc_gyr_mag(imu):
	acc, gyro, mag = imu.getMotion9()

	# print ("Acc:", "{:+7.3f}".format(acc[0]), "{:+7.3f}".format(acc[1]), "{:+7.3f}".format(acc[2]),)
	# print (" Gyr:", "{:+8.3f}".format(gyro[0]), "{:+8.3f}".format(gyro[1]), "{:+8.3f}".format(gyro[2]),)
	# print (" Mag:", "{:+7.3f}".format(mag[0]), "{:+7.3f}".format(mag[1]), "{:+7.3f}".format(mag[2]))
	# yaw=atan2(mag[0],mag[1])*180/pi
	return acc,gyro,mag

def magn_calib(imu,duration) :
	print("Magnetometer Calibration")
	print("________________________")
	time.sleep(5)
	mx,my,mz=[],[],[]
	print("Rotate the IMU through all possible orientations in 1 sec")
	time.sleep(1)
	t0=time.time()
	while time.time()-t0 < duration :
		mag=acc_gyr_mag(imu)[2]
		mx.append(mag[0])
		my.append(mag[1])
		mz.append(mag[2])
	print("Copying in a file")
	mag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '{}_magnetometer_calibration.txt'.format(str(imu)))
	mag_data_file=os.path.join(os.path.dirname(os.path.abspath(__file__)), '{}_magnetometer_data_for_calibration.txt'.format(str(imu)))
	mag_f=open(mag_file,"w")
	mag_data_f=open(mag_data_file,"w")
	mag_data_f.write(mx)
	mag_data_f.write(my)
	mag_data_f.write(mz)
	mag_data_f.close()
	for i in range(len(mx)) :
		mag_f.write('{} | {} | {}'.format(mx[i],my[i],mz[i]))
	print("Data copied")
	params = [0.,0.,0.,0.]
	myResult = optimize.leastsq(res_sphere, params, args=(np.array(mx),np.array(my),np.array(mz)) )
	ox, oy, oz, r = myResult[0]
	mag_f.close()
	print("Offsets : {} | {} | {}".format(ox,oy,oz))
	print("Magnetometer Calibration ends")
	return ox,oy,oz

def res_sphere(p,x,y,z):
	""" residuals from sphere fit """
	a,b,c,r = p                             # a,b,c are center x,y,c coords to be fit, r is the radius to be fit
	distance = np.sqrt( (x-a)**2 + (y-b)**2 + (z-c)**2 )
	err = distance - r                 # err is distance from input point to current fitted surface
	return err

def show_calib_mag(imu) :
	mag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '{}_magnetometer__data_for_calibration.txt'.format(str(imu)))
	mag_f=open(mag_file,'r')
	mx,my,mz=mag_f.readlines()
	mag_f.close()
	fig=plt.figure()
	ax=plt.axes(projection="3d")
	ax.scatter(np.array(mx),np.array(my),np.array(mz),c='b')
	plt.show()

if __name__=="__main__" :
	imu=init_lsm()
	ox,oy,oz=magn_calib(imu,60)
	show_calib_mag(imu)