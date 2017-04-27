#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import laser_geometry

import rospy

def callback(laser_scan_msg):
    point = laser_geometry.LaserProjection()
    pc2 = point.projectLaser(laser_scan_msg, -1.0, point.ChannelOption.DEFAULT)
    pub = rospy.Publisher('~output', PointCloud2, queue_size=10)
    pub.publish(pc2)


if __name__ == '__main__':
    rospy.init_node('laser_scan_to_point_cloud2')
    sub = rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()
