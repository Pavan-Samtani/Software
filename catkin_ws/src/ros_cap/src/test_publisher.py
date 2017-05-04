#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped


def main():
    rospy.init_node('test_publisher')
    rospy.loginfo('test_publisher')
    # Base cmd
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    # Publish each sec
    rate = rospy.Rate(1)
    msg = Twist2DStamped()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.get_rostime()     
        msg.omega = 0.5		
        msg.v = 0.5
        base_pub.publish(msg)
        rate.sleep()



if __name__ == '__main__':
    main()

