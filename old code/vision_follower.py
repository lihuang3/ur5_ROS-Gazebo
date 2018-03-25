#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)

  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # END BRIDGE
    # BEGIN HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # END HSV
    # BEGIN FILTER
    lower_red = np.array([ 0,  100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    #BEGIN FINDER
    M = cv2.moments(mask)
    print M
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    # END FINDER
    # BEGIN circle
      cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
      #END circle

    cv2.namedWindow("window", 1)
    cv2.imshow("window", image )
    cv2.waitKey(5)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
