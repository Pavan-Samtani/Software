#!/usr/bin/env python
#fx168.86429674181502
#fy170.69143299098607
#hx3.7
#hy3.2


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([160,120,120])
upper_red = np.array([179,255,255])
lower_yellow = np.array([20,150,120])
upper_yellow = np.array([30,255,255])

class Detector():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('/duckiebot/camera_node/image/rect',Image, self._process_image)

   
        #Publicar a topicos
        self.publicar=rospy.Publisher('imagen', Image, queue_size=1)
        
        self.pub_punto=rospy.Publisher('Puntos', Point, queue_size=1)
        
        self.pub_wheels=rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 400


    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)


        # Filtrar colores de la imagen en el rango utilizando 
        #mask = cv2.inRange(image, lower_limit, upper_limit)

        mask=cv2.inRange(frameHSV, lower_yellow, upper_yellow)
        
        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        msg =    self.bridge.cv2_to_imgmsg(segment_image, "bgr8")


        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        img_out = cv2.erode(mask, kernel, iterations = 3)
        
        #Operacion morfologica dilate
        imagen_final =cv2.dilate(img_out, kernel, iterations = 2)

        ShitImage, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        x0=0;y0=0;w0=0;h0=0        

        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)

            #Filtrar por area minima
            if w*h > self.min_area:

                
                if w*h>w0*h0:
                    x0=x;y0=y;w0=w;h0=h
                    cv2.rectangle(frame, (x0,y0), (x0+w0,y0+h0), (0,0,0), 2)
                    #z=(168.86429674181502*3.7)/w0
                    z=(170.69143299098607*3.2)/h0
                    Punto=Point(x0+w0/2,y0+h0/2,z)

                    #Punto medio
                       

                    #Publicar Point center de mayor tamanio
                    self.pub_punto.publish(Punto)
        msg1 =self.bridge.cv2_to_imgmsg(frame, "bgr8")
        #Publicar frame
        self.publicar.publish(msg1)

def main():

    rospy.init_node('Detector')

    Detector()

    rospy.spin()

if __name__ == '__main__':
    main()


    
        

