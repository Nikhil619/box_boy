#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

pub = rospy.Publisher('/scan2', LaserScan, queue_size = 10)
scann = LaserScan()

def callback(msg):
    print len(msg.ranges)
    print("FRONT SIDE: ", msg.ranges)
    
    """
    for i in range(len(msg.ranges)):
        if str(msg.ranges[i]) != 'inf':
           print(i)
    """

def listener():
    rospy.init_node('scan2', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
