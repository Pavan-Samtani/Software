#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
import time

def main():
 rospy.init_node('test_subscriber')
 rospy.loginfo('test_subscriber')



 
 # Subscriber for joint states
 sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
 rospy.spin()

def freno(msg):
    while msg.buttons[2]==1:  #Freno
        msg_t.v=0

def Movilidad(msg):
    if abs(float(msg.axes[1]))<0.25: #Precisar el avance
        msg_t.v=0
    msg_t.omega=msg.axes[0]*100.0
    msg_T.v=msg.axes[1]

def aceleracion(msg):  #Aceleracion recta
    while msg.buttons[3]==1:
        if float(msg_t.v)>=1.15:
            msg_t.v=1
        msg_t.v = msg_t.v + 0.15
        freno(msg)
        time.sleep(1)

def iniciar(msg):
    freno(msg)
    aceleracion(msg)
    Movilidad(msg)


	
def process_callback(msg):
 
 base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
 msg_t = Twist2DStamped()
 iniciar(msg)
 base_pub.publish(msg_t)

		


if __name__ == '__main__':
 main()

	



