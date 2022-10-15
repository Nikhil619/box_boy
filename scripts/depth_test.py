import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import cv2

bridge = CvBridge()

def callback2(msg_depth):
    print("looping")
    try:
      # The depth image is a single-channel float32 image
      # the values is the distance in mm in z axis
      cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "8UC1")
      cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
      cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
      cv_image_resized = cv2.resize(cv_image_norm, desired_shape, interpolation = cv2.INTER_CUBIC)
      depthimg = cv_image_resized
      cv2.imshow("Image from my node", depthimg)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)


def listener():
    rospy.init_node('okokak', anonymous=True)
    depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,callback2,queue_size=1)
    #self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
