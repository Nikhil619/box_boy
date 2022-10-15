#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time
import threading


move = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size =1)
target = None

def avoid(time_):
    move.linear.x = -0.3
    move.angular.z = 0.0
    pub.publish(move)
    time.sleep(time_)
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    time.sleep(0.5)   

def rotate(angle):
    move.linear.x = 0
    if angle < 0:
       turn_vel = -0.785
    elif angle >= 0:
       turn_vel = 0.785
    move.angular.z = turn_vel
    pub.publish(move)
    #time.sleep(1)
    #move.linear.x = 0.0
    #move.angular.z = 0.0
    #pub.publish(move)
    #time.sleep(0.1)    

def callback_rot(msg):
    global target
    print(msg.linear.x)
    if msg.linear.x < 0.5 and msg.linear.x > -0.3: 
       if msg.angular.z >= 0.8 or msg.angular.z <= -0.8:
          print(msg.angular.z, msg.linear.x)
          y = threading.Thread(target=rotate, args=(msg.angular.z,))
          y.start()
    #rotate(msg.angular.z)
    #target = msg.angular.z
    #time.sleep(1)

def callback(msg):

  #If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
  if msg.ranges[0] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[18] <0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[36] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[54] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[72] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[90] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[630] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[648] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[666] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[684] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[702] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  elif msg.ranges[719] < 0.4: #1
     if msg.ranges[320] < 0.7 and msg.ranges[380] < 0.7 and msg.ranges[260] < 0.7 and msg.ranges[440] < 0.7 and msg.ranges[200] < 0.7:
        x = threading.Thread(target=avoid, args=(4,))
        x.start()
  #print(msg.ranges[0])
  
#sub2 = rospy.Subscriber('/cmd_vel', Twist, callback_rot, queue_size =1)
while not rospy.is_shutdown():
    rospy.init_node('rot_node')
    sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1) 
    sub2 = rospy.Subscriber('/cmd_vel', Twist, callback_rot, queue_size =1)
    """
    print(target)
    if target != None:
       if target > 0.55 or target < -0.55:
          y = threading.Thread(target=rotate, args=(target,))
          y.start()
    """
    
    time.sleep(1)



