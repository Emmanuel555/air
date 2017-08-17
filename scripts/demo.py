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
from mavros import setpoint as SP
from std_msgs.msg import Header
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix


class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 1.5
        self.errorDx = 0.0
        self.errorDy = 0.0
        self.armed = False
        self.pitch = 0.0
        self.roll = 0.0
        self.timeout = 180
        self.count = 0
        self.error_updated = [False, False]
        self.descent = False

        rospy.wait_for_service('mavros/cmd/arming', 30)

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        rospy.Subscriber("error_dx", Float32, self.error_dx)
        rospy.Subscriber("error_dy", Float32, self.error_dy)
        self.pub_spt = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_att = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.pub_thr = rospy.Publisher('mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
        self.pub_lpe = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.pub_vel = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        rospy.wait_for_service('mavros/cmd/command', 30)

        self._srv_cmd_long = rospy.ServiceProxy('mavros/cmd/command', CommandLong, persistent=True)

        self.rate = rospy.Rate(50) # 10hz
        self.has_global_pos = True
        self.local_position = PoseStamped()
        self.forward = True;

        while not rospy.is_shutdown():

            if (self.forward):
                self.x = self.x + 0.01 # move forward
            else:
                self.x = self.x - 0.01

            if (self.x >= 25):
                self.forward = False
            if (self.x <= -1):
                self.forward = True

            self.test_posctl()
            #self.test_attctl()
            #self.lpe()
            #self.vel()
            self.arm()

            self.rate.sleep()

    #
    # General callback functions used in tests
    #
    def vel(self):
        if (True):
       # if (self.error_updated[0] == True and self.error_updated[1] == True):
            self.error_updated[0] = False
            self.error_updated[1] = False
            pos = TwistStamped()
            pos.header = Header()
            pos.header.frame_id = "local_origin"
            pos.twist.linear.x = 0
            pos.twist.linear.y = 0
            pos.twist.linear.z = 0.2

            pos.twist.angular.x = 0
            pos.twist.angular.y = 0
            pos.twist.angular.z = 0

            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_vel.publish(pos)


    def lpe(self):
        if (True):
       # if (self.error_updated[0] == True and self.error_updated[1] == True):
            self.error_updated[0] = False
            self.error_updated[1] = False
            pos = PoseStamped()
            pos.header = Header()
            pos.header.frame_id = "local_origin"
            pos.pose.position.x = self.errorDx/1000
            pos.pose.position.y = self.errorDy/1000
            pos.pose.position.z = self.local_position.pose.position.z

            # For demo purposes we will lock yaw/heading to north.
            # pos.pose.orientation = self.local_position.pose.orientation
            q = quaternion_from_euler(0, 0, 0)
            pos.pose.orientation.x = q[0]
            pos.pose.orientation.y = q[1]
            pos.pose.orientation.z = q[2]
            pos.pose.orientation.w = q[3]

            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_lpe.publish(pos)

    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def error_dx(self, msg):
        self.errorDx = msg.data
        self.error_updated[0] = True

    def error_dy(self, msg):
        self.errorDy = msg.data
        self.error_updated[1] = True

    def pid_thr(self, desired_z):
        z = self.local_position.pose.position.z
        #k1 = 0.1
        #u = 0.614 + k1 * (desired_z - z)
        #if u > 0.615:
        #    u = 0.615
        #if u < 0.6108:
        #    #u = 0.6108
        #    u = 0.61075
        #print u
        u = 0.612

        return u

    def is_at_position(self, x, y, z, offset):
        rospy.logdebug("current position %f, %f, %f" %
                       (self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return linalg.norm(desired - pos) < offset

    def arm(self):
        if self.count < self.timeout:
            # FIXME: arm and switch to offboard
            # (need to wait the first few rounds until PX4 has the offboard stream)
            if not self.armed and self.count > 5:
                self._srv_cmd_long(False, 176, False,
                                   1, 6, 0, 0, 0, 0, 0)
                # make sure the first command doesn't get lost
                time.sleep(1)

                self._srv_cmd_long(False, 400, False,
                                   # arm
                                   1, 0, 0, 0, 0, 0, 0)

                self.armed = True
            self.count = self.count + 1
            self.rate.sleep()

    def test_posctl(self):
        # print "posctl: "
        # set a position setpoint
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"
        pos.pose.position.x = self.x
        pos.pose.position.y = self.y
        pos.pose.position.z = self.z

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)

        # update timestamp for each published SP
        pos.header.stamp = rospy.Time.now()
        self.pub_spt.publish(pos)
        #self.helper.bag_write('mavros/setpoint_position/local', pos)

    def test_attctl(self):
        """Test offboard attitude control"""

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            print "waiting for global_pos"
            self.rate.sleep()

        # set some attitude and thrust
        att = PoseStamped()
        att.header = Header()
        att.header.frame_id = "base_footprint"
        att.header.stamp = rospy.Time.now()
        yaw_degree = 0
        yaw = radians(yaw_degree)
        att.pose.position.z = self.z
        self.roll = (self.errorDx-75)/1000 # scale the dx
        self.pitch = (-self.errorDy+60)/1000 # scale the dy
        self.roll = 0.2 * self.roll
        self.pitch = 0.2 * self.pitch
        #print self.roll

        #self.roll = radians(self.errorDx)
        #self.pitch = radians(-self.errorDy)
        quaternion = quaternion_from_euler(self.roll, self.pitch, yaw)
        #quaternion = quaternion_from_euler(roll, pitch, yaw)
        att.pose.orientation = Quaternion(*quaternion)

        throttle = Float64()
        #throttle.data = 0.615
        throttle.data = self.pid_thr(5)
        if abs(self.roll) > 0.03 and self.local_position.pose.position.z > 1.0:
            throttle.data = 0.613
        if abs(self.pitch) > 0.03 and self.local_position.pose.position.z > 1.0:
            throttle.data = 0.613

        # update timestamp for each published SP
        att.header.stamp = rospy.Time.now()

        self.pub_att.publish(att)
        #self.helper.bag_write('mavros/setpoint_attitude/attitude', att)
        self.pub_thr.publish(throttle)
        #self.helper.bag_write('mavros/setpoint_attitude/att_throttle', throttle)

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)

    node = SetpointPosition()

    rospy.spin()
