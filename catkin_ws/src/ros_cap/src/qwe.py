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

	
def process_callback(msg):
 
 base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
 msg_t = Twist2DStamped()
 msg_t.v = msg.axes[1]*5
 msg_t.omega = msg.axes[0]*5
 

 
 base_pub.publish(msg_t)
