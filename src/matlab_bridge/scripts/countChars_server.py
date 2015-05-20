#!/usr/bin/env python

from mimason.srv import *
import rospy
from cv_bridge import CvBridge, CvBridgeError

def handle_countChars(req):
    print "so very here"
    print req.s
    # print "Returning number of characters in " + req.s
    return countCharsResponse(len(req.s))

def countChars_server():
    rospy.init_node('countChars_server')
    s = rospy.Service('countChars', countChars, handle_countChars)
    print "Ready to count chars"
    rospy.spin()

if __name__ == "__main__":
    countChars_server()
