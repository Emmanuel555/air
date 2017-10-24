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

import low_pass

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
from mavros_msgs.msg import PositionTarget, RCIn
from sensor_msgs.msg import NavSatFix, Range, LaserScan
#from gazebo_msgs.msg import ModelStates

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
        self.fakeX = 0
        self.fakeY = 0
        self.spX = 0
        self.spY = 0
        self.rcX = 0
        self.rcY = 0

        self.rcX_trim = 1519
        self.x_max = 60.0/100.0 #max x position for UAV to chase. convert from 5cm to metres. x_max is in metres
        self.rcX_max = 500.0 #max range of rc from centre trim of 1500
        self.scalingX = self.x_max/self.rcX_max # scaling factor for rcIn to posX

        self.rcY_trim = 1519
        self.y_max = 100.0/100.0 #max x position for UAV to chase. convert from 5cm to metres. x_max is in metres
        self.rcY_max = 500.0 #max range of rc from centre trim of 1500
        self.scalingY = self.y_max/self.rcY_max # scaling factor for rcIn to posX

        self.freq = 10
        self.lpDx = low_pass.lowpassfilter(self.freq, 0.6)
        self.lpDy = low_pass.lowpassfilter(self.freq, 0.6)


        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/setpoint_raw/target_local", PositionTarget, self.setpoint_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        rospy.Subscriber("mavros/distance_sensor/hrlv_ez4_pub", Range, self.error_lpZ, queue_size=1)
        rospy.Subscriber("mavros/rc/in", RCIn, self.updateRCIn, queue_size=1)
        #rospy.Subscriber("teraranger0/laser/scan", LaserScan, self.range_callback)
        rospy.Subscriber("error_dx", Float32, self.error_dx)
        #rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_pose)
        rospy.Subscriber("error_dy", Float32, self.error_dy)
        rospy.Subscriber("error_dz", Float32, self.error_dz)
        rospy.Subscriber("roll", Float32, self.error_roll)
        rospy.Subscriber("pitch", Float32, self.error_pitch)

        self.pub_lpe = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=1)
        self.rate = rospy.Rate(self.freq) # 20hz
        self.has_global_pos = True
        self.local_position = PoseStamped()
        self.setpointX = 0


        # self.filterDx = 0
        # self.filterDy = 0

        while not rospy.is_shutdown():

            self.rate.sleep()
            self.lpe(self.errorDx)
            # self.lpe(self.errorDx) #for using lsq errorDx = 0
            # self.lpe(self.fakeX) #for using fakeX toggled by RCIn[7]

    #
    # General callback functions used in tests
    #

    def lpe(self, errorDx):
        if (self.error_updated[0] == True and self.error_updated[1] == True):
            self.error_updated[0] = False
            self.error_updated[1] = False
            pos = PoseStamped()
            pos.header = self.local_position.header #Header()
            pos.header.frame_id = "local_origin"
            ## pixhawk is using NED convention i.e. x (front/north), y (right,east), z(down)
            ## however lsq x-y-z is left, front, down
            #pos.pose.position.x = errorDx
            #pos.pose.position.y = self.errorDy
            #pos.pose.position.z = self.z
            pos.pose.position.y = -errorDx
            pos.pose.position.x = self.errorDy
            # pos.pose.position.x = pos.pose.position.x - self.fakeY #to activate slider
            pos.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            q = self.local_position.pose.orientation
            # print q
            # euler[2] = self.errorDz
            # print q
            # euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            #q = quaternion_from_euler(0, 0, -self.errorDz+np.pi/2)
            #self.errorDz = np.pi/4
            q = quaternion_from_euler(np.pi+self.roll, self.pitch, np.pi/2+self.errorDz) #x,y,z, 'zyx order'
            pos.pose.orientation.x = q[0]
            pos.pose.orientation.y = q[1]
            pos.pose.orientation.z = q[2]
            pos.pose.orientation.w = q[3]
            #pos.pose.orientation.x = q[0]
            #pos.pose.orientation.y = q[1]
            #pos.pose.orientation.z = q[3]
            #pos.pose.orientation.w = q[2]

            # print q
            # pos.pose.orientation = self.local_position.pose.orientation
            # q = self.local_position.pose.orientation

            # euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            # print 'roll ', euler[0], '\t pitch ', euler[1], '\t yaw ', euler[2]

            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_lpe.publish(pos)

    def position_callback(self, data):
        self.local_position = data
        # self.z = 0
        # self.lpe()

    def setpoint_callback(self, data):
        self.spX = data.position.y #because of ned to enu shit
        self.spY = -data.position.x

    def global_position_callback(self, data):
        self.has_global_pos = True

    def range_callback(self, msg):
        self.z = msg.ranges[0]
        if self.z >= 5:
            self.z = 0

    def error_roll(self, msg):
        self.roll = msg.data

    def error_pitch(self, msg):
        self.pitch = msg.data

    def gazebo_pose(self, msg):
        # print msg.pose[2].position.x
        self.errorDx = msg.pose[2].position.x
        self.error_updated[0] = True

    def error_dx(self, msg):
        #self.errorDx = msg.data
        self.errorDx = self.lpDx.update_filter(msg.data)
        self.error_updated[0] = True

    def error_dy(self, msg):
        #self.errorDy = msg.data
        self.errorDy = self.lpDy.update_filter(msg.data)
        self.error_updated[1] = True

    def error_dz(self, msg):
        self.errorDz = msg.data

    def error_lpZ(self, msg):
        self.z = msg.range

    def updateRCIn(self, msg):
        self.rcX = msg.channels[7] - self.rcX_trim
        tempX = self.rcX * self.scalingX #absolute difference in x from the setpointX
        self.fakeX = self.spX - tempX # +ve tempX implies
        #print self.spX

        self.rcY = msg.channels[5] - self.rcY_trim
        self.fakeY = self.rcY * self.scalingY #absolute difference in x from the setpointX
        #print self.spX


if __name__ == '__main__':
    rospy.init_node('vision_test_node')

    node = VisionPosition()

    rospy.spin()
