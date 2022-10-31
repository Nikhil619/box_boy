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

def node():
	global frontiers,mapData
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not 

	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	

	rateHz = rospy.get_param('~rate',100)
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
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
                print((str(len(centroids))+" frontiers available, ", centroids))
                
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
