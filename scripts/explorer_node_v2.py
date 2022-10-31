#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from autonav.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import threading


# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]

def callBack(data):
    global frontiers
    frontiers=[]
    for point in data.points:
        frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def get_odom(msg):
    global x, y, z, yaw, orientation_q
    orientation_q = msg.pose.pose.position
    orientation_y = msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x, y, z = orientation_q.x, orientation_q.y, orientation_q.z
    yaw = orientation_y.z

def movebase_client(x_goal, y_goal, yaw_goal):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_goal #3.6014963
    goal.target_pose.pose.position.y = y_goal #-1.156947665
    goal.target_pose.pose.orientation.z = yaw_goal

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server DOWN ;/ ")
    else:
        return client.get_result()

def node():
    global frontiers,mapData
    rospy.init_node('assigner', anonymous=False)
    
    # fetching all parameters
    map_topic= rospy.get_param('~map_topic','map')
    info_radius= rospy.get_param('~info_radius',1.0)                    #this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not 

    frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')    

    rateHz = rospy.get_param('~rate',100)
    
    rate = rospy.Rate(rateHz)
#-------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
    rospy.Subscriber('/odom', Odometry, get_odom)
#---------------------------------------------------------------------------------------------------------------
        
# wait if no frontier is received yet 
    while len(frontiers)<1:
        pass
    centroids=copy(frontiers)    
#wait if map is not received yet
    while (len(mapData.data)<1):
        pass    

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
    while not rospy.is_shutdown():   
        centroids=copy(frontiers)
        #print(str(len(centroids))+" frontiers available, ", centroids)
        print((x,y, "robo pos"))
        
        nearest_check = []
        for i in centroids:
            dist = math.sqrt((x-i[0])**2+(y-i[1])**2)
            if dist < 1:
               nearest_check.append(10000000)
            else:
               nearest_check.append(dist)
  
        nearest_dist = min(nearest_check)
        goal = centroids[nearest_check.index(nearest_dist)]
        x_goal, y_goal = goal[0], goal[1]
        print((x_goal, y_goal))

        try:
           movebase_client(x_goal, y_goal, 0.9)
        except:
           print("chaalu bro")
#------------------------------------------------------------------------- 
        rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
