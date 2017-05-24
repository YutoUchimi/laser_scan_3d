#!/usr/bin/env python

import sys
import os
import roslib
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
roslib.load_manifest('laser_assembler')

global pub
global assemble_scans2

def callback(laser_scan_msg):
    try:
        response = assemble_scans2(rospy.get_rostime()-rospy.Duration(interval), rospy.get_rostime())
        print "Got cloud with %u points" % len(response.cloud.data)
        pub.publish(response.cloud)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    global pub
    global assemble_scans2
    if len(sys.argv) < 2:
        sys.exit(exit_code)
    else:
        interval = (float)(sys.argv[1])
    rospy.init_node("assemble_scans_client")
    rospy.wait_for_service("assemble_scans2")
    rospy.sleep(interval)
    assemble_scans2 = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher("~output", PointCloud2, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()
