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
        response = assemble_scans2(rospy.get_rostime()-rospy.Duration(assemble_interval), rospy.get_rostime())
        rospy.loginfo("Got cloud with %u points" % len(response.cloud.data))
        pub.publish(response.cloud)
    except rospy.ServiceException, e:
        rospy.logerror("Service call failed: %s" % e)


if __name__ == '__main__':
    global pub
    global assemble_scans2
    rospy.init_node("assemble_scans_client")
    assemble_interval = rospy.get_param('~assemble_interval', 10.0)
    queue_size = rospy.get_param('~queue_size', 10)
    rospy.wait_for_service("assemble_scans2")
    rospy.sleep(assemble_interval)
    assemble_scans2 = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher("~output", PointCloud2, queue_size=queue_size)
    sub = rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()
