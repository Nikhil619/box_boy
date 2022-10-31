#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sensor_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import threading
import tf

pub = rospy.Publisher('/move_base_simple2/goal', PoseStamped, queue_size = 2)

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


def callback(msg):
    x_g = msg.pose.position.x
    y_g = msg.pose.position.y
    w_g = msg.pose.orientation.w  
    movebase_client(x_g,y_g,w_g)
    print("goal reached")

def listener():
    rospy.init_node('move_base_tros', anonymous=True)
    sub = rospy.Subscriber('/move_base_simple2/goal', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
