#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae, Vladimir Ermakov
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
#    - Use mavros.setpoint module for topics

import rospy
import thread
import threading
import time
import mavros

from numpy import linalg
import numpy as np

from math import *
from mavros.utils import *
from mavros.param import *
from mavros import setpoint as SP
from std_msgs.msg import Header
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix, Range, LaserScan
from gazebo_msgs.msg import ModelStates

class VisionPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.errorDx = 0.0
        self.errorDy = 0.0
        self.errorDz = 0.0
        self.armed = False
        self.pitch = 0.0
        self.roll = 0.0
        self.timeout = 180
        self.count = 0
        self.error_updated = [False, False]
        self.descent = False
        self.z = 0

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        rospy.Subscriber("teraranger0/laser/scan", LaserScan, self.range_callback)
        rospy.Subscriber("error_dx", Float32, self.error_dx)
        # rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_pose)
        rospy.Subscriber("error_dy", Float32, self.error_dy)
        rospy.Subscriber("error_dz", Float32, self.error_dz)

        self.pub_lpe = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)

        self.rate = rospy.Rate(20) # 20hz
        self.has_global_pos = True
        self.local_position = PoseStamped()

        while not rospy.is_shutdown():
            self.rate.sleep()
            self.lpe()

    #
    # General callback functions used in tests
    #

    def lpe(self):
        if (self.error_updated[0] == True and self.error_updated[1] == True):
            self.error_updated[0] = False
            self.error_updated[1] = False
            pos = PoseStamped()
            pos.header = Header()
            pos.header.frame_id = "local_origin"
            pos.pose.position.x = self.errorDy
            pos.pose.position.y = self.errorDx
            pos.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            q = self.local_position.pose.orientation
            # print q
            # euler[2] = self.errorDz
            # print q
            # euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            q = quaternion_from_euler(0, 0, self.errorDz)
            pos.pose.orientation.x = q[0]
            pos.pose.orientation.y = q[1]
            pos.pose.orientation.z = q[3]
            pos.pose.orientation.w = q[2]
            # print (euler_from_quaternion(q))[2]
            # pos.pose.orientation = self.local_position.pose.orientation
            # q = self.local_position.pose.orientation

            # euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            # print 'roll ', euler[0], '\t pitch ', euler[1], '\t yaw ', euler[2]

            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_lpe.publish(pos)

    def position_callback(self, data):
        self.local_position = data
        # self.lpe()

    def global_position_callback(self, data):
        self.has_global_pos = True

    def range_callback(self, msg):
        self.z = msg.ranges[0]
        if self.z >= 5:
            self.z = 0

    def gazebo_pose(self, msg):
        # print msg.pose[2].position.x
        self.errorDy = msg.pose[2].position.x
        self.error_updated[1] = True

    def error_dx(self, msg):
        self.errorDx = msg.data
        self.error_updated[0] = True

    def error_dy(self, msg):
        self.errorDy = msg.data
        self.error_updated[1] = True

    def error_dz(self, msg):
        self.errorDz = msg.data

if __name__ == '__main__':
    rospy.init_node('vision_test_node', anonymous=True)

    node = VisionPosition()

    rospy.spin()
