#! /usr/bin/env python
import numpy as np
from sklearn.linear_model import RANSACRegressor

ransac = RANSACRegressor()
#**********************************************************************************************************************************
def fit_ransac(R,rmax):
	L = len(R)
	X = []
	Y = []
	for i in range (0,L):
		if (R[i]<rmax): #and (R[i]>0.11):
			r = R[i]*100.0
			th = i*(np.pi/180.0)
			X.append(r*np.cos(th))
			Y.append(r*np.sin(th))
	L = len(X)
	X = np.reshape(np.array(X),(L,1))
	Y = np.reshape(np.array(Y),(L,1))
	reg = ransac.fit(X,Y)
	x1 = 0
	x2 = 1
	X_m = np.array([[x1], [x2]]) #np.reshape(np.array([x1, x2]),(2,1))
	y1,y2 = reg.predict(X_m)
	m = (y2-y1)/(x2-x1)
	b = y2-m*x2
	return(m,b)
#**********************************************************************************************************************************
def steer_control(m,b,th_min):
	d_ref = 25.0 #25.0 #30.0#45.0
	Kx = 0.06*2.0 #0.06
	Kth = 1.25*2.0 #1.25 
	gamma = th_min-(3.0*np.pi/2.0)
	d = abs(b/np.sqrt(m**2+1.0))
	ex = (d-d_ref)
	th = -np.arctan(m) 
	u = int(90.0-np.arctan(Kx*ex+Kth*th)*(180.0/np.pi))
	return(u,d)
#**********************************************************************************************************************************
def measure_D(r_min,th_min):
	if (th_min<4.7124):
		D_est = 2.0*r_min*np.sin(4.7124-th_min)
	else: D_est = 0.0
	return D_est 

