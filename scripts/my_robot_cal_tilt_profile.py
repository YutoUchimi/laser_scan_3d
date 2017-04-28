#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG)

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import *
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller]
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print_usage()

    cmd = LaserTrajCmd()
    controller   =    sys.argv[1]
    cmd.header   =    rospy.Header(None, None, None)
    cmd.profile  = "blended_linear"
    #cmd.pos      = [1.0, .26, -.26, -.7,   -.7,   -.26,   .26,   1.0, 1.0]
    d = .025
    #cmd.time     = [0.0, 0.4,  1.0, 1.1, 1.1+d,  1.2+d, 1.8+d, 2.2+d, 2.2+2*d]

<<<<<<< HEAD
    dur = (float)(sys.argv[2]);
    # cmd.position = [-0.7,  1.2, -0.7]
    cmd.position = [-0.4,  1.0, -0.4]
    cmd.time_from_start = [0.0,  dur, dur+1.0]
=======
    dur = 29;
    # cmd.position = [-0.7,  1.2, -0.7]
    cmd.position = [-0.4, -0.3, 0.9, 1.0, -0.4]
    cmd.time_from_start = [0.0, 2.0, dur-2.0, dur, dur+1.0]
>>>>>>> 1bc45fe8c21eb86ff830089e6276837e35e9b185
    cmd.time_from_start = [rospy.Duration.from_sec(x) for x in cmd.time_from_start]
    cmd.max_velocity = 10
    cmd.max_acceleration = 30

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Pos: %s ' % ','.join(['%.3f' % x for x in cmd.position])
    print '  Time: %s' % ','.join(['%.3f' % x.to_sec() for x in cmd.time_from_start])
    print '  MaxRate: %f' % cmd.max_velocity
    print '  MaxAccel: %f' % cmd.max_acceleration

    rospy.wait_for_service(controller + '/set_traj_cmd')

    s = rospy.ServiceProxy(controller + '/set_traj_cmd', SetLaserTrajCmd)
    resp = s.call(SetLaserTrajCmdRequest(cmd))

    print 'Command sent!'
    print '  Resposne: %f' % resp.start_time.to_sec()
