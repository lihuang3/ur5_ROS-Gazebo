#!/usr/bin/env python
# BEGIN ALL
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', 
                                      Image, self.image_callback)
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    cv2.namedWindow("window", 1)
    cv2.imshow("window", image)
    cv2.waitKey(5)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
