#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw',
                                      Image, self.image_callback)
  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # END BRIDGE
    # BEGIN HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # END HSV
    # BEGIN FILTER
    lower_red = numpy.array([ 0,  100, 100])
    upper_red = numpy.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # END FILTER
    masked = cv2.bitwise_and(image, image, mask=mask)
    cv2.namedWindow("window1", 1)
    cv2.imshow("window1", mask )
    cv2.waitKey(5)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
