#!/usr/bin/env python

from mimason.srv import *
import rospy
import os
import cv2
import scipy.io as sio
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud


class Recognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.update_im = False
        self.update_pc = False
        self.cv_image = None
        self.cloud = None
        self.count = 0
        # rospy.init_node('recognizer')
        # rospy.Subscriber("redo_recognition", bool, self.update_redo)
        rospy.Subscriber("cloud", PointCloud, self.update_point_cloud)
        rospy.Subscriber("image_raw", Image, self.update_rgb)


    def update_point_cloud(self,update):
        if(self.update_pc):
            self.cloud = update
            self.update_pc = False
    def update_rgb(self,im_message):
        if(self.update_im):
            try:
                self.cv_image = bridge.imgmsg_to_cv2(im_message, "bgr8")
                self.update_im = False
            except CvBridgeError, e:
                print e
    def recognize(self,req):
        #get updated image and point cloud
        self.update_im = True
        self.update_pc = True
        self.count += 1

        #wait some time amd ensure image and PointCloud updated
        while not (self.update_pc and self.update_im):
            print 'in loop'
            time.sleep(.1)
        cv2.imwrite('image_to_recognize.png',self.cv_image)
        # os.system("matlab demo_sds.m")
        os.system("ls")
        rr = sio.loadmat('recognition_results.mat')
        # mask = rr['mask']/float(np.shape(mask)[0]
        mask = np.zeros((8,8))
        print np.shape(mask)
        categories = rr['categories']
        confidence = rr['confidence']
        return recognizeObjectsResponse(self.bridge.cv2_to_imgmsg(mask, "mono8"),categories,confidence)


def main(args):
  r = Recognizer()
  print "initializing"
  rospy.init_node('recognizeObjects_server')
  s = rospy.Service('recognizeObjects', recognizeObjects, r.recognize)
  print "Ready to recognize"
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
