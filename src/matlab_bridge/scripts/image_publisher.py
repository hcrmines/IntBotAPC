#!/usr/bin/env python
import roslib
roslib.load_manifest('mimason')
import sys
import rospy
import cv2
import os
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class image_publisher:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size = 1)
        self.bridge = CvBridge()
        # cv2.namedWindow("Image window", 1)
        # print "ere"
        self.cv_image = cv2.imread("/home/hcrws1/catkin_ws/src/mimason/scripts/astley.jpg");
        # print self.cv_image
        self.rate = rospy.Rate(1)
    def callback(self):
        os.system("echo ive been working on the maaatlab all the livelong day")
        #   cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        (rows,cols,channels) = self.cv_image.shape
        # print self.cv_image.shape
        # cv2.imshow('image',self.cv_image)


        if cols > 60 and rows > 60 :
            cv2.circle(self.cv_image, (random.randint(100,200),random.randint(100,200)), random.randint(10,100), random.randint(10,100))
            print "ere"
            # cv2.imshow("Image window", self.cv_image)
            # cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError, e:
            print e

def main(args):
  rospy.init_node('image_publisher', anonymous=True)
  ic = image_publisher()

  # try:
  #   rospy.spin()
  # except KeyboardInterrupt:
  #   print "Shutting down"
  while not rospy.is_shutdown():
      ic.callback()
      ic.rate.sleep()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
