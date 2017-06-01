#!/usr/bin/env python
#fx336.58763787613367
#fy342.678617163186
#hx3.7
#hy3.2


import rospy
# import rospkg

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy

import numpy as np

class Controller():

    def __init__(self):
        self.sub_punto = rospy.Subscriber('Puntos',Point, self.process_joy)
        self.sub = rospy.Subscriber('/duckiebot/wheels_driver_node/possible_cmd', Twist2DStamped,self.command)
        self.pub_wheels=rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.punto=None
        self.command=None 


    def process_joy(self,point):
        self.punto=point
        
    def comando(self,msg):
        msg_t=Twist2DStamped()
        x=float(self.punto.x)
        y=self.punto.y
        z=self.punto.z
        if float(z)<=15:
            if x>=160 and x<=230:
                msg_t.omega=1
                msg_t.v=msg.v
                self.pub_wheels.publish(msg_t)
            if x>=90 and x<160:
                msg_t.omega=-1
                msg_t.v=msg.v
                self.pub_wheels.publish(msg_t)
        self.pub_wheels.publish(msg)


    def main():

        rospy.init_node('Controller')

        Controller()

        rospy.spin()

    if __name__ == '__main__':
        main()

