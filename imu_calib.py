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
	mag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'imu_magnetometer_calibration.txt')
	mag_f=open(mag_file,"w")
	for i in range(len(mx)) :
		mag_f.write('{} | {} | {}\n'.format(mx[i],my[i],mz[i]))
	print("Data copied")
	params = [0.,0.,0.,0.]
	myResult = optimize.leastsq(res_sphere, params, args=(np.array(mx),np.array(my),np.array(mz)) )
	ox, oy, oz, r = myResult[0]
	mag_f.close()
	print("Offsets : {} | {} | {}\n".format(ox,oy,oz))
	print("Magnetometer Calibration ends")
	return ox,oy,oz

def acc_calib(imu,duration) :
	print("Accelerometer Calibration")
	print("_________________________")
	print("Place the Raspberry on a plane surface")
	time.sleep(1)
	print("Waiting for calibration ({} seconds)".format(duration))
	ax,ay,az=[],[],[]
	t0=time.time()
	while time.time()-t0 < duration :
		acc=imu.getMotion9()[0]
		ax.append(acc[0])		
		ay.append(acc[1])
		az.append(acc[2])
	print("Copying in a file")
	acc_file=os.path.join(os.path.dirname(os.path.abspath(__file__)),'imu_accelerometer_calibration.txt')
	acc_f=open(acc_file,'w')
	for k in range(len(ax)) :
		acc_f.write("{} | {} | {}\n".format(ax[k],ay[k],az[k]))
	print("Data copied")
	ox,oy,oz=np.mean(ax),np.mean(ay),np.mean(az)-9.81
	acc_f.close()
	print("Offsets : {} | {} | {}".format(ox,oy,oz))
	print("Accelerometer Calibration ends")
	return ox,oy,oz

def res_sphere(p,x,y,z):
	""" residuals from sphere fit """
	a,b,c,r = p                             # a,b,c are center x,y,c coords to be fit, r is the radius to be fit
	distance = np.sqrt( (x-a)**2 + (y-b)**2 + (z-c)**2 )
	err = distance - r                 # err is distance from input point to current fitted surface
	return err

def show_calib_mag(imu) :
	mag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'imu_magnetometer_calibration.txt')
	mag_f=open(mag_file,'r')
	mx,my,mz=[],[],[]
	data=mag_f.readlines()
	mag_f.close()
	for k in range(len(data)) :
		l=data[k].split(" | ")
		x=l[0]
		y=l[1]
		z=l[2]
		mx.append(x)
		my.append(y)
		mz.append(z)
	fig=plt.figure()
	ax=plt.axes(projection="3d")
	ax.scatter(np.array(mx),np.array(my),np.array(mz),c='b')
	plt.show()

if __name__=="__main__" :
	imu=init_lsm()
	# omx,omy,omz=magn_calib(imu,180)
	oax,oay,oaz=acc_calib(imu,180)
	offsets_file=os.path.join(os.path.dirname(os.path.abspath(__file__)),'offset_file.txt')
	off_f=open(offsets_file,'w')
	# off_f.write("Magnetometer Offsets are : {} | {} | {}\n\n".format(omx,omy,omz))
	off_f.write("Accelerometer Offsets are : {} | {} | {}".format(oax,oay,oaz))
	off_f.close()
	show_calib_mag(imu)