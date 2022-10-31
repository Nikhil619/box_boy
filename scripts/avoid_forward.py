#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import threading


move = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size =1)

def avoid(time_):
    move.linear.x = -0.3
    move.angular.z = 0.0
    pub.publish(move)
    time.sleep(time_)
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    time.sleep(0.5)    

def callback(msg):

  #If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
  if msg.ranges[0] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[18] <0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[36] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[54] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[72] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[90] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[630] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[648] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[666] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[684] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[702] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  elif msg.ranges[719] < 0.4: #1
     if msg.ranges[320] > 0.7 and msg.ranges[380] > 0.7 and msg.ranges[260] > 0.7 and msg.ranges[440] > 0.7 and msg.ranges[200] > 0.7:
        x = threading.Thread(target=avoid, args=(8,))
        x.start()
  print((msg.ranges[0]))
  

while not rospy.is_shutdown():
    rospy.init_node('rot_node')
    sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1) 
    time.sleep(1)



