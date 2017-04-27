#!/usr/bin/env python

import roslib
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
roslib.load_manifest('laser_assembler')


def callback(laser_scan_msg):
    rospy.wait_for_service("assemble_scans2")
    pub = rospy.Publisher("~output", PointCloud2, queue_size=10)

    try:
        assemble_scans2 = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        response = assemble_scans2(rospy.Time(0,0), rospy.get_rostime())
        print "Got cloud with %u points" % len(response.cloud.data)
        pub.publish(response.cloud)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node("assemble_scans_client")

    sub = rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()
