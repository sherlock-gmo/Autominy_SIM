#!/usr/bin/env python
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
cap = cv.VideoCapture(0)
#======================================================
#					VIDEO CAPTURE
#======================================================
def video_cap():
	_, imagen0 = cap.read()	    							  							# Original
	image_msg = bridge.cv2_to_imgmsg(imagen0, "bgr8")
	Image_pub.publish(image_msg)				#Publica el mensaje de imagen en Image_topic
#====================================================================
#						PRINCIPAL
#====================================================================
if __name__ == '__main__':
	try:
		print ('*** Nodo Image_node inicializado ***')
		rospy.init_node('Image_node', anonymous=True)	
		Image_pub = rospy.Publisher('/app/camera/rgb/image_raw', Image, queue_size=8) 		
		#rate = rospy.Rate(5) 					# Frecuencia a la que publica = 7.4 hz
		while not rospy.is_shutdown():	# Ejecuta el bucle mientras no se presiona ctrl+C
			video_cap()
			#rate.sleep()									# Delay adecuado para publicar a 7.4 Hz
	except rospy.ROSInterruptException:
		pass
