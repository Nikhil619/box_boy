import rospy
from geometry_msgs.msg import Twist
import time

rospy.init_node('lastforward')
rate=rospy.Rate(4)
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)

move=Twist()
def move_x_secs(secs):
    move.linear.x = -0.1
    pub.publish(move)
    time.sleep(secs)
    move.linear.x = 0
    time.sleep(0.5)

while not rospy.is_shutdown():
    move_x_secs(2.5)
