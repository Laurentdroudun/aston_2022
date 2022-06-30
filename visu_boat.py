import matplotlib.pyplot as plt
import numpy as np
import os
from roblib import *

if __name__=="__main__" :
	log_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log.txt')
	log_f=open(log_file,'r')
	data=log_f.readlines()
	x,y,speed,s_heading,yaw,x_wind,y_wind,wind_dir=[],[],[],[],[],[],[],[]
	x_tr,y_tr=[],[]
	delta_r,delta_s=[],[]
	for k in range(len(data)) :
		l=data[k].split(" | ")
		x.append(l[0])
		y.append(l[1])
		speed.append(l[2])
		s_heading.append(l[3])
		yaw.append(l[4])
		x_wind.append(l[5])
		y_wind.append(l[6])
		wind_dir.append(l[7])
		a_ap=np.linalg.norm(np.array([[l[5]],[l[6]]]))
		xtr,ytr=-l[2]*np.cos((l[4]*pi/180+pi)%2*pi)+a_ap*np.cos(l[7]+l[4]),-l[2]*np.sin((l[4]*pi/180+pi)%2*pi)+a_ap*np.sin(l[7]+l[4]*pi/180)
		x_tr.append(xtr)
		y_tr.append(ytr)
		delta_r.append(l[8])
		delta_s.append(l[9])
	plt.figure("Visu")
	plt.plot(x,y)
