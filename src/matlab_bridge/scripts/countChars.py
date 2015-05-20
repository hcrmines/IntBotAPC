#!/usr/bin/env python

from mimason.srv import *
import rospy

def handle_countChars(req):
    print "Returning number of characters in " + req
    return countCharsResponse(len(req))

def countChars_server():
    rospy.init_node('countChars_server')
    s = rospy.Service('countChars', countChars, handle_countChars)
    print "Ready to count chars"
    rospy.spin()

if __name__ == "__main__":
    countChars_server()
