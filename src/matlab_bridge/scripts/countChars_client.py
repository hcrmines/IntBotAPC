#!/usr/bin/env python

import sys
import rospy
from mimason.srv import *

def countChars_client(s):
    rospy.wait_for_service('countChars')
    try:
        count_chars = rospy.ServiceProxy('countChars', countChars)
        # print count_chars
        resp1 = count_chars(s)
        # print 'ere'
        return resp1.count
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [str]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        s = str(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s counts"%s
    print "There are " + str(countChars_client(s)) + " characters in '" + s
    "' "
