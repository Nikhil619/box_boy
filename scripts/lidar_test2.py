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

command_velocity = Twist()
#lidar_data = LaserScan()
lidar_data= []

def rotate(angle):
    move.linear.x = 0
    if angle < 0:
       turn_vel = -0.7
    elif angle >= 0:
       turn_vel = 0.7
    move.angular.z = turn_vel
    pub.publish(move)  

def callback_rot(msg):
    global command_velocity
    command_velocity = msg

def callback(msg):
    #print(msg)
    del lidar_data[:]
    lidar_data.append(msg.ranges)
 
#sub2 = rospy.Subscriber('/cmd_vel', Twist, callback_rot, queue_size =1)
def node(): 
        while not rospy.is_shutdown():
            rospy.init_node('rot_node')
            sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1) 
            sub2 = rospy.Subscriber('/cmd_vel', Twist, callback_rot, queue_size =1)

            if len(lidar_data) > 0:
               print(command_velocity.linear.x)
               print(len(lidar_data[0]))
               if lidar_data[0][180] < 0.4 or lidar_data[0][150] < 0.4 or lidar_data[0][210] < 0.4:                
                  print(command_velocity.angular.z, command_velocity.linear.x)
                  x = threading.Thread(target=rotate, args=(command_velocity.angular.z,))
                  x.start()  

               elif lidar_data[0][0] < 0.4 or lidar_data[0][30] < 0.4 or lidar_data[0][330] < 0.4:                
                  print(command_velocity.angular.z, command_velocity.linear.x)
                  x = threading.Thread(target=rotate, args=(command_velocity.angular.z,))
                  x.start()        
                
               elif command_velocity.linear.x < 0.225 and command_velocity.linear.x > -0.085: 
                  if command_velocity.angular.z >= 0.5 or command_velocity.angular.z <= -0.5:
                     print(command_velocity.angular.z, command_velocity.linear.x)
                     y = threading.Thread(target=rotate, args=(command_velocity.angular.z,))
                     y.start()    

            time.sleep(1)

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 

