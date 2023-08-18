#! /usr/bin/env python
import cv2
import numpy as np

#**********************************************************************************************************************************
def tip(imagenN):
	H=np.array([[-7.98362236e-02,-4.79765416e-01,1.23982766e+02],[1.05493081e-03,-1.61957424,3.77026220e+02],[7.48177877e-06,-4.86995945e-03,1.0]]) 
	imagenH = cv2.warpPerspective(imagenN, H, (200,300),borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)) 
	return imagenH
#**********************************************************************************************************************************
def roi_zone(x):
	assert (x>=0) and (x<=199), 'x out of limits'
	if (x>135) and (x<=199): #130
		y = int(round(-1.6875*x+499.8125))
	if (x>=64) and (x<=135): #69 130 
		y = 272 #280
	if (x>=0) and (x<64): #69
		y = int(round(1.7375*x+160.0))
	return y
#**********************************************************************************************************************************
def vec_create(x,stride):
	j = 0
	xv = []
	for i in range(0,2*stride+1):
		if ((-1)**i==-1): j = j+1
		xv.append(x+j*(-1)**i)
	return xv
#**********************************************************************************************************************************
def line_detector(imagen0,x1,l,side):
	K = True
	stride = 8
	y1 = roi_zone(x1)
	x1v = vec_create(x1,stride)
	while (K==True):
		if (y1+stride>299): m = 299-y1
		else: m = stride
		for j in range(y1+m,y1-stride,-1):
			for i in x1v:
				if imagen0[j][i]==255:
					x1 = i
					y1 = j
					K = False
					break
			x1v = vec_create(x1,stride)
			if (K==False): break
		if (K==True):
			x1 = x1-1*side
			y1 = roi_zone(x1)
	x2 = x1
	x2v = vec_create(x2,stride)
	for j in range(y1-1,y1-l,-1):
		for i in x2v:
			if imagen0[j][i]==255:
				x2 = i
				y2 = j
				K = False
				break
		x2v = vec_create(x2,stride)			
	return x1,y1,x2,y2
