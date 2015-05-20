#!/usr/bin/env python
import roslib
roslib.load_manifest('mimason')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_publisher:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        cv2.namedWindow("Image window", 1)
    def callback(self,data):
        cv_image = cv2.imload("astley.jpg",0)
        #   cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        #   cv2.imshow('image',cv_image)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print e

def main(args):
  ic = image_publisher()
  rospy.init_node('image_publisher', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
