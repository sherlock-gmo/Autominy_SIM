import cv2
import h5py
import numpy as np
from scipy.signal import convolve2d, correlate2d
import skimage.measure

#	INFORMACION A PRIORI QUE SE REQUIERE PARA RECONSTRUIR EL MODELO:
# 1.- Tamanos de los kernels de cada etapa de pooling.
#	2.- Funciones de activacion de cada capa
#	3.- El padding y el stride de cada capa de convolucion/pooling

#**************************************************************************
#**************************************************************************
#**************************************************************************
# 				OBTENCION DE LOS PESOS Y BIAS
def get_KBW(h5_path):
	# Obtiene los pesos y los bias de un modelo entrenado y guardado en formato HDF5
	# Devuelve una lista (variable list) con los tensores (variables np). Cada indice corresponde a una capa 
	# h5_path - Variable str con la ruta del modelo entrenado
	hf = h5py.File(h5_path, 'r')
	#print(list(hf))
	print(list(hf['model_weights']))
	#print(list(hf['model_weights/dense_1/dense1_1/kernel:0']))
	print('********************')
	Nd = 0
	Nc = 0		
	for i in list(hf['model_weights']):
		if (i[0:7]=='conv2d_'):
			Nc = Nc+1
		if (i[0:6]=='dense_'):
			Nd = Nd+1
	print('Numero de capas de convolucion =',Nc)
	print('Numero de capas de la red Feedfoward =',Nd)
	#__________________________________Obtiene los Pesos y los Bias de las capas de convolucion
	Bc = []	# Bias de cada capa
	Wc = []	# Pesos de cada capa
	pathWc = ''
	pathBc = ''
	for i in range (Nc):
		pathBc = 'model_weights/conv2d_'+str(i+1)+'/conv2d_'+str(i+1)+'/bias:0'
		pathWc = 'model_weights/conv2d_'+str(i+1)+'/conv2d_'+str(i+1)+'/kernel:0'
		bc = np.array(hf[pathBc])
		wc = np.array(hf[pathWc])
		Bc.append(bc)
		Wc.append(wc)
		print('Conv2d_'+str(i+1)+'/Bc ',bc.shape)
		print('Conv2d_'+str(i+1)+'/Wc ',wc.shape)
	#__________________________________Obtiene los Pesos y los Bias de la RN feedfoward
	Bd = []	# Bias de cada capa
	Wd = []	# Pesos de cada capa
	pathWd = ''
	pathBd = ''
	for i in range (Nd):
		pathBd = 'model_weights/dense_'+str(i+1)+'/dense_'+str(i+1)+'/bias:0'
		pathWd = 'model_weights/dense_'+str(i+1)+'/dense_'+str(i+1)+'/kernel:0'
		bd = np.array(hf[pathBd])
		wd = np.array(hf[pathWd])
		Bd.append(bd)
		Wd.append(wd)
		print('Dense_'+str(i+1)+'/Bd ',bd.shape)
		print('Dense_'+str(i+1)+'/Wd ',wd.shape)
	print('********************')
	hf.close()
	return Bc, Wc, Bd, Wd
#**************************************************************************
#**************************************************************************
#**************************************************************************
#				FUNCIONES DE ACTIVACION
#________________________RELU
def af_relu(v_in): 
	v_out = (0.5)*np.add(v_in,abs(v_in))
	return v_out
#_______________________SIGMOID
def af_sigmoid(v_in): 
	v_minus= (-1.0)*v_in
	v_den =np.add(1.0,np.exp(v_minus))
	v_list = []
	for i in v_den:
		v_list.append(1.0/i)
	v_out = np.array(v_list)
	return v_out 
#_______________________SOFTMAX
def af_softmax(v_in):
	v_exp = np.exp(v_in)
	S = np.sum(v_exp)
	v_out = (1.0/S)*v_exp
	return v_out
#**************************************************************************
#**************************************************************************
#**************************************************************************
# 				CAPA COMPLETAMENTE CONECTADA	
def nn_Dlayer(v_in,B,W,af):
	h = np.matmul(v_in,W)+B
	if(af == 'relu'):
		v_out = af_relu(h)
	if(af == 'sigmoid'):
		v_out = af_sigmoid(h)
	if(af == 'softmax'):
		v_out = af_softmax(h)
	if(af == 'none'):
		v_out = h
	return v_out
#**************************************************************************
#**************************************************************************
#**************************************************************************
# 				CAPA DE CONVOLUCION
def nn_Clayer(Im_in,B,W,af,padding,stride):
	Xi,Yi,D = Im_in.shape
	Xk,Yk,_,N = W.shape
	s1 = stride[0]
	s2 = stride[1]
	if (padding == 0): 
		m = 'valid'
		Xi = int((Xi-Xk+s1)/s1)
		Yi = int((Yi-Yk	+s2)/s2)
	else: m = 'same'
	H = np.zeros((Xi,Yi,N))
	h = np.zeros((Xi,Yi))
	for i in range(N):
		for j in range(D):
			kernel = W[:,:,j,i]
			imagen = Im_in[:,:,j]
			#imagenF =  convolve2d(imagen,kernel,mode=m)
			imagenF =  correlate2d(imagen,kernel,mode=m)[::s1, ::s2]
			h = h+imagenF
		H[:,:,i] = h+B[i]*np.ones((Xi,Yi))
		h = np.zeros((Xi,Yi))
	if(af == 'relu'):
		Im_out = af_relu(H)
	#for k in range(N):
		#cv2.imshow("Show by CV2",(1.0/np.max(Im_out[:,:,k]))*Im_out[:,:,k])
		#cv2.waitKey(0)
	return Im_out
#**************************************************************************
#**************************************************************************
#**************************************************************************
# 				POOLING
def nn_Pool(Im_in,N,m):
	if (m=='max_P'): pooling = np.max
	X,Y,D = Im_in.shape
	Xo = int(X/N)
	Yo = int(Y/N)
	Im_out = np.zeros((Xo,Yo,D))
	for i in range(D): 	
		j = Im_in[:,:,i]
		Im_out[:,:,i] = skimage.measure.block_reduce(j, (N,N), pooling)
	return Im_out
#**************************************************************************
#**************************************************************************
#**************************************************************************
# 				FLATTEN
def nn_flatt(Im_in):
	N,M,L = Im_in.shape
	D = N*M*L
	Im_out = np.reshape(Im_in,(1,D))
	return Im_out
