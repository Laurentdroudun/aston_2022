# import spidev	
import time 
import argparse 
import sys 
from math import * 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize
import os

def show_calib_mag() :
	mag_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'imu_magnetometer_calibration.txt')
	mag_f=open(mag_file,'r')
	mx,my,mz=[],[],[]
	mx_calib,my_calib,mz_calib=[],[],[]
	data=mag_f.readlines()
	mag_f.close()
	offset_mag_x=-7.959210618259436
	offset_mag_y=-21.521659859117926
	offset_mag_z=-11.650658574455132
	for k in range(len(data)) :
		l=data[k].split(" | ")
		x=float(l[0])
		x_calib=x-offset_mag_x
		y=float(l[1])
		y_calib=y-offset_mag_y
		z=float(l[2])
		z_calib=z-offset_mag_z
		mx.append(x)
		my.append(y)
		mz.append(z)
		mx_calib.append(x_calib)
		my_calib.append(y_calib)
		mz_calib.append(z_calib)
	print(mx)
	fig=plt.figure()
	ax=plt.axes(projection="3d")
	ax.scatter(np.array(mx),np.array(my),np.array(mz),c='b',marker='o',s=0.2)
	ax.scatter(np.array(mx_calib),np.array(my_calib),np.array(mz_calib),c='r',marker='o',s=0.2)
	plt.show()

if __name__=="__main__" :
	show_calib_mag()