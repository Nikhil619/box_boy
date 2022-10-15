#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import math

pub = rospy.Publisher('/scan2', LaserScan, queue_size = 10)
#scann = LaserScan()

def callback(msg):
    kinect_scans_center = []
    x = []
    y = []
    theta = []
    distance_from_center = []
    final_ranges = []
    theta_ranges_fin = []
    dist_from_lidar = 18.7
    k = 0
    for i in range(-320, 320):
        m = msg.ranges[k]
        angle = i*30/320
        if angle ==0:
            angle = 0.1
        x.append(m*math.cos(math.sqrt(angle**2)*math.pi/180))
        y.append(m*math.sin(math.sqrt(angle**2)*math.pi/180))
        theta.append(angle)
        distance_from_center.append(m*math.cos(math.sqrt(angle**2)*math.pi/180) + 0.1925)
        k = k+1
    for i in range(len(distance_from_center)):
        theta_fin_perp = math.atan(distance_from_center[i]/y[i])
        theta_fin = math.pi/2 - theta_fin_perp
        theta_ranges_fin.append(theta_fin)
        final_ranges.append(distance_from_center[i]/math.cos(theta_fin))
        print(theta_fin)
    msg.ranges = final_ranges
    msg.angle_max = 0.54
    msg.angle_min = -0.47

    pub.publish(msg)

def listener():
    rospy.init_node('scan2', anonymous=True)
    sub = rospy.Subscriber('laserscan_kinect/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
