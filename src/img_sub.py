#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
#try:
#    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
#except ValueError:
#    pass  # do nothing!
#sys.path.append('../')




bridge = CvBridge()


def image_callback(ros_image):

  print('got an image')
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "32FC1")
    depth_Img = np.array(cv_image, dtype=np.float32) * 0.001
    #cv2.normalize(depth_Img, depth_Img, 0, 1, cv2.NORM_MINMAX)

  except CvBridgeError as e:
      print(e)
  cv2.imshow("Image window", depth_Img)
  cv2.waitKey(3)
  rospy.sleep(1)  
 

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("vCam/depth/image",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)