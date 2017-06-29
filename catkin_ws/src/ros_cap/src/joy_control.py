#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
import time

class joy_control():

    def __init__(self):
        self.sub = rospy.Subscriber('/duckiebot/joy', Joy, self.process_callback)
        self.pub_wheels=rospy.Publisher('/duckiebot/wheels_driver_node/possible_cmd', Twist2DStamped, queue_size=1)

	
    def process_callback(self,msg):
         msg_t = Twist2DStamped()
         if abs(float(msg.axes[1]))<0.3: #Precisar el avance
            msg_t.v=0
         else:
            msg_t.v=msg.axes[1]
         if abs(float(msg.axes[0]))<0.3:
            msg_t.omega=0
         else:
            msg_t.omega=msg.axes[0]*10
         self.pub_wheels.publish(msg_t)

def main():

    rospy.init_node('joy_control')

    joy_control()

    rospy.spin()

if __name__ == '__main__':
    main()
 
 
