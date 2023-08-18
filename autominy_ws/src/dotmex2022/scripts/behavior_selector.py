#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Int16, Bool

path_libs = '/home/dotmex/dotMEX_Autominy_SIM/autominy_ws/src/dotmex2022/scripts/libs'
import sys
sys.path.insert(1, path_libs)
from line_finder import tip, line_detector
from lib_parking_functions import steer_control, fit_ransac, measure_D
from lib_model_builder import get_KBW
from lib_nn_maneuver import get_NNmaneuver
_,_,B_d,W_d = get_KBW(path_libs+'/NN_maneuver.hdf5')

#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
class sensors_processing(object):
#----------------------------------------------------------------------------------------------------------------
#																	INIT
#----------------------------------------------------------------------------------------------------------------
	def __init__(self):
		# V. Vision
		self.x1_h = 120
		self.FT = 0
		self.ex = 0
		self.th = 0.0
		# V. YOLO
		self.v_max = -400
		self.Ped = False
		self.SemRed = False
		# V. IMU
		self.FTY = True
		self.yaw0 = 0.0
		self.yaw_h = 0.0
		self.Dyaw = 0.0
		#V. Lidar
		self.step = 0
		self.Ev = False
		#V. Parking
		self.Park = False
		self.flag = False
		rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback_Vis)
		rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback_YOLO)	
		rospy.Subscriber('/scan', LaserScan, self.callback_Lidar)
		rospy.Subscriber('/AutoNOMOS_mini/imu', Imu, self.callback_Imu)
		rospy.Subscriber('/parking', Bool, self.callback_Parking)
		self.Vpub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=15)				 
		self.Spub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=15)

#----------------------------------------------------------------------------------------------------------------
#																	CALLBACKS
#----------------------------------------------------------------------------------------------------------------
#																	VISION
	def callback_Vis(self, data_vis):
		bridge = CvBridge()
		imagen0 = bridge.imgmsg_to_cv2(data_vis, "bgr8") 	
		self.maneuver(imagen0)

#----------------------------------------------------------------------------------------------------------------
#																	YOLO
	def callback_YOLO(self, data_yolo):
		self.Ped = False
		self.SemRed = False
		self.Car = False
		id_class = data_yolo.bounding_boxes
		for i in range(len(id_class)):
			id_number = id_class[i].id
			area = (id_class[i].xmax-id_class[i].xmin)*(id_class[i].ymax-id_class[i].ymin)
			cx = int(id_class[i].xmin+(id_class[i].xmax-id_class[i].xmin)/2.0)
			cy = int(id_class[i].ymin+(id_class[i].ymax-id_class[i].ymin)/2.0)
			# Senales de transito
			if (id_number == 15) and (area>=2167): self.SemRed = False 	# Sem Verde
			if (id_number == 17) and (area>=2167): self.SemRed = True		# Sem Rojo
			if (id_number == 13) and (area>=3515): self.v_max = -400		# lim 50km_h
			if (id_number == 14) and (area>=3515): self.v_max = -800		# lim 100km_h
			#if (id_number == 9) and (area>=3544):  self.NoEst = True		# No estacionarse

			# Objetos del camino
			if (id_number == 4) and (area>=5800) and (50<=cx<=550) and (195<=cy<=360): self.Ped = True		# Peaton 
			if (id_number == 0) and (area>=4313) and (250<=cx<=380): self.Car = True 											# Carro

		"""
		id		class
		------------
		0			carro
		1			camion
		2			bicicleta
		3			moto
		4			peaton
		5			perro
		6			gato
		7			caballo
		8			alto
		9			noest
		10		20km
		11		30km
		12		40km
		13		50km
		14		100km
		15		sem_verde
		16		sem_amarillo
		17		sem_rojo
		18		sem_NA
		19		tren
	"""
#----------------------------------------------------------------------------------------------------------------
#																	LIDAR
	def callback_Lidar(self, data_lidar):
		Rlim = 1.2
		self.Obj = False
		#self.r_min = 3.0 
		#self.r270 = 3.0
		self.R = []
		for r in data_lidar.ranges:
			if (r>=3.0): r = 3.0 #or (r<=0.11)
			self.R.append(r)
		self.r_min = np.amin(self.R) 
		self.r270 = self.R[269]
		R0_i = self.R[0:12] 			#11 deg
		R0_d = self.R[349:359] 
		R0 = np.concatenate((R0_i,R0_d))
		r0 = np.amin(R0)
		if (r0<=Rlim): 
			self.Obj = True
#----------------------------------------------------------------------------------------------------------------
#																	IMU
	def callback_Imu(self, data_imu):
		f_imu = 50.0
		h = 1.0/f_imu
		gz = data_imu.angular_velocity.z
		yaw = (self.yaw_h+h*gz)	
		if (yaw>60*np.pi) or (yaw<-60*np.pi):
			yaw = 0.0
			self.yaw0 = 0.0
		self.yaw_h = yaw
		if (self.FTY==True): self.yaw0 = yaw
		self.Dyaw = (self.yaw0-yaw)*(180.0/np.pi)
#----------------------------------------------------------------------------------------------------------------
#																	PARKING
	def callback_Parking(self, data_parking):
		self.Park = data_parking.data

#----------------------------------------------------------------------------------------------------------------
#																	CONTROLLERS
#----------------------------------------------------------------------------------------------------------------

	def maneuver(self, imagen0):
		value = 1

		if (abs(self.th)<=0.2618): self.Curve = False
		else: self.Curve = True
		if (self.Obj==True) and (self.Car==True) and (self.Curve==False): 
			self.Ev = True
			self.FTY = False
		"""
		Entradas:
		[Est Curve Ev Obj Ped SemRed] -- Renglones de In_bin
		Salidas
		[Alto SeguirCarril Evasion Estacionamiento] -- Renglones de Out_bin
		"""
		x = np.array([int(self.Park),int(self.Curve),int(self.Ev),int(self.Obj),int(self.Ped),int(self.SemRed)])
		y = get_NNmaneuver(B_d,W_d,x)
		value = np.argmax(y)

		#_______________________________________________________________________________________
		#_______________________________________________________________________________________
		#_______________________________________________________________________________________
		if (value == 0):
			print('-----------ALTO-----------')
			v = 0
			u = 90
			print('steering ',u)
			print('speed ',v)
			print('************************')
			self.Vpub.publish(v) 
			self.Spub.publish(u)

		#_______________________________________________________________________________________
		if (value == 1):
			l = 60 
			x_ref = 120
			y1 = 0
			y2 = 0
			imagenG = cv2.cvtColor(imagen0,cv2.COLOR_BGR2GRAY) 					
			imagenT = tip(imagenG)
			_,imagenB = cv2.threshold(imagenT,75,255,cv2.THRESH_BINARY)
			imagenF = cv2.Sobel(imagenB,cv2.CV_8U,1,0, ksize=3)
			if (self.FT<=30):
				print('-----------INCORPORAMIENTO AL CARRIL-----------')
				x1 = 180
				self.FT = self.FT+1
			else: 
				print('-----------SEGUIMIENTO DEL CARRIL-----------')
				x1 = self.x1_h
			x1,y1,x2,y2 = line_detector(imagenF,x1,l,True)
			self.x1_h = x1
			kx = 0.05616094 
			kth = 0.16484354
			# vrpm	R				Q				Kx						Kth
			# 800		20		0.05*I	0.0487392   0.15340431
			# 800		15		0.05*I	0.05616094	0.16484354
			# 800		10		0.05*I	0.06855525  0.18258912
			self.ex = x1-x_ref
			self.th = np.arctan2(x2-x1,l)
			u = int(round(90-np.arctan(kx*self.ex+kth*self.th)*(180/np.pi))) 
			v = self.v_max
			print('steering ',u)
			print('speed ',v)
			print('************************')
		 	#Visualizacion
			imagenS = cv2.cvtColor(imagenF,cv2.COLOR_GRAY2BGR)
			imagenS = cv2.circle(imagenS,(x1,y1),3,(0, 0, 255),-1)
			imagenS = cv2.circle(imagenS,(x2,y2),3,(0, 0, 255),-1)
			imagenS = cv2.line(imagenS, (x1,y1), (x2,y2), (0, 0, 255), 2) 
			cv2.imshow('homografia',imagenS)	
			#cv2.moveWindow("homografia", 700,30)
			cv2.waitKey(1)
			self.Vpub.publish(v) 
			self.Spub.publish(u)

		#_______________________________________________________________________________________
		if (value==2):
			D = 30.0 #28.0
			d = 0.0
			ky = 5.0
			print('-----------EVASION-----------')
			v = -400
			if (self.step == 0):
				u = 180
				if (self.Dyaw<=-D): self.step = self.step+1
			if (self.step == 1):
				u = 0
				if (self.Dyaw>=-d): self.step = self.step+1
			if (self.step == 2):
				u = 90.0+ky*self.Dyaw
				if (self.r_min>=0.4) and (self.r270>=0.5):
					self.Ev = False
					self.FT = 0
					self.FTY = True
					self.step = 0 
			print('step ',self.step)
			print('Dyaw ',self.Dyaw)
			print('************************')
			self.Vpub.publish(v) 
			self.Spub.publish(u)

		#_______________________________________________________________________________________
		if (value==3):
			print('-----------ESTACIONAMIENTO-----------')
			rmax = 3.0
			D = 38.0  #36.0#34.0 #29.79
			r_min = np.amin(self.R)
			th_min = np.argmin(self.R)*(np.pi/180.0) 
			m,b = fit_ransac(self.R,rmax) 		
			r180 = np.amin(self.R[149:209])
			R0_i = self.R[0:29]
			R0_d = self.R[329:359]
			R0 = np.concatenate((R0_i,R0_d))
			r0 = np.amin(R0)

			if (self.step == 0):
				u,d = steer_control(m,b,th_min)
				v = -100
				Dest = measure_D(r_min,th_min) 
				#print('d ',d)
				#print("r_est ",r_est)
				#print("Dest ",Dest)
				if (Dest>0.8):
					self.flag =True
				if (self.flag==True) and (self.r270<0.5): 
					self.FTY = False
					self.step = self.step+1
			if (self.step == 1):
				print('Inicio de la maniobra')
				u = 0
				v = 100 #250
				if (self.Dyaw<=-D):
					self.step = self.step + 1
			if (self.step == 2):
				u = 180
				v = 100 #250
				if (self.Dyaw>=0.0) or (r180<=0.35): #0.42 ###
					self.step = self.step + 1
			if (self.step == 3):
				if (self.Dyaw<-1): u = 0
				if (self.Dyaw>=-1): u = 90
				v = -100 #-250
				if (r0<=0.5): #0.6
					self.step = self.step + 1
			if (self.step == 4):
				if (self.Dyaw<-3): self.step = 2
				if (self.Dyaw>=-3):
					print('Fin de la maniobra')
					u = 90
					v = 0

			# Visualizacion
			x1 = -200
			x2 = 200
			y1_r = int(299.0-(m*x1+b))
			y2_r = int(299.0-(m*x2+b))
			x1_r = x1+299
			x2_r = x2+299
			x1_min = int(100.0*r_min*np.cos(th_min)+299.0)
			y1_min = int(299.0-100.0*r_min*np.sin(th_min))
			imagenR = np.zeros((600,600,3))
			font = cv2.FONT_HERSHEY_SIMPLEX 																	
			imagenR =cv2.line(imagenR, (299,299),(329,299), (255,0,0), 2) #ejeX
			imagenR = cv2.putText(imagenR, 'X', (331, 299), font, 0.4, (255,255,255), 1, cv2.LINE_AA)	
			imagenR =cv2.line(imagenR, (299,299),(299,269), (0,0,255), 2) #ejeY
			imagenR = cv2.putText(imagenR, 'Y', (299,267), font, 0.4, (255,255,255), 1, cv2.LINE_AA)
			imagenR =cv2.line(imagenR, (x1_r,y1_r),(x2_r,y2_r), (0,125,255), 2)
			imagenR =cv2.line(imagenR, (299,299),(x1_min,y1_min), (255,255,0), 2)
			imagenR = cv2.circle(imagenR,(299,299),2,(255, 255, 255),-1)
			i = 0
			for j in self.R:
				r_l = j*100.0
				th_l = (i)*(np.pi/180.0)
				x = int(r_l*np.cos(th_l)+299.0)
				y = int(299.0-r_l*np.sin(th_l))
				imagenR = cv2.circle(imagenR,(x,y),2,(0, 0, 255),-1)
				i = i+1
			cv2.imshow('lidar',imagenR)	
			#cv2.moveWindow("lidar", 800,400)
			cv2.waitKey(1)

			print('u ',u)
			print('step ',self.step)
			print('****************************')
			self.Vpub.publish(v)
			self.Spub.publish(u)

#**********************************************************************************************************************************
#**********************************************************************************************************************************
#**********************************************************************************************************************************
if __name__ == '__main__':
	print("Nodo inicializado: behavior_selector.py")			
	rospy.init_node('Behavior Selector',anonymous=True)	
	sensors_processing()
	rospy.spin()








