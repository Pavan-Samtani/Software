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
 if abs(float(msg.axes[1]))<0.5: #Precisar el avance
        msg_t.v=0
 if abs(float(msg.axes[0]))<0.75:
        msg_t.omega=0
 msg_t.omega=msg.axes[0]
 msg_t.v=msg.axes[1]
 base_pub.publish(msg_t)

		


if __name__ == '__main__':
 main()

	



