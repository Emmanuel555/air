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
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix, Range

class VisionPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.errorDx = 0.0
        self.errorDy = 0.0     
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
        rospy.Subscriber("mavros/px4flow/ground_distance", Range, self.range_callback)
        rospy.Subscriber("error_dx", Float32, self.error_dx)
        rospy.Subscriber("error_dy", Float32, self.error_dy)

        self.pub_lpe = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)

        self.rate = rospy.Rate(50) # 10hz
        self.has_global_pos = True
        self.local_position = PoseStamped() 

        while not rospy.is_shutdown():
            #self.lpe()
            #self.vel()
            #self.arm()
            
            self.rate.sleep()

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
            pos.pose.position.x = self.errorDy/1000
            pos.pose.position.y = self.errorDx/1000
            pos.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            pos.pose.orientation = self.local_position.pose.orientation
        
            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_lpe.publish(pos)    
    
    def position_callback(self, data):
        self.local_position = data
        self.lpe()

    def global_position_callback(self, data):
        self.has_global_pos = True

    def range_callback(self, msg):
        self.z = msg.range

    def error_dx(self, msg):
        self.errorDx = msg.data
        self.error_updated[0] = True

    def error_dy(self, msg):
        self.errorDy = msg.data
        self.error_updated[1] = True

if __name__ == '__main__':
    rospy.init_node('vision_test_node', anonymous=True)

    node = VisionPosition()

    rospy.spin()


