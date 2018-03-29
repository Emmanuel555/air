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
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
from tf import TransformBroadcaster, TransformListener
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import PositionTarget, RCIn
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Range, LaserScan, Imu
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

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
        self.x_max = 30.0/100.0 #max x position for UAV to chase. convert from 5cm to metres. x_max is in metres
        self.rcX_max = 500.0 #max range of rc from centre trim of 1500
        self.scalingX = self.x_max/self.rcX_max # scaling factor for rcIn to posX

        self.rcY_trim = 1519
        self.y_max = 100.0/100.0 #max x position for UAV to chase. convert from 5cm to metres. x_max is in metres
        self.rcY_max = 500.0 #max range of rc from centre trim of 1500
        self.scalingY = self.y_max/self.rcY_max # scaling factor for rcIn to posX
        self.imuCalibrated = False
        self.imuOffset = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu = Imu()
        self.last_twist = TwistStamped()
        self.pub_imu_tf = TransformBroadcaster()
        self.pub_rplidar_tf = TransformBroadcaster()
        self.sub_robot_tf = TransformListener()
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size=1)
        self.local_position = PoseStamped()
        self.lpos = PoseStamped()
        # rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        # rospy.Subscriber("mavros/setpoint_raw/target_local", PositionTarget, self.setpoint_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        # rospy.Subscriber("mavros/distance_sensor/hrlv_ez4_pub", Range, self.error_lpZ, queue_size=1)
        # rospy.Subscriber("mavros/rc/in", RCIn, self.updateRCIn, queue_size=1)
        # rospy.Subscriber("teraranger0/laser/scan", LaserScan, self.range_callback)
        # rospy.Subscriber("error_dx", Float32, self.error_dx)
        rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_pose)
        # rospy.Subscriber("error_dy", Float32, self.error_dy)
        rospy.Subscriber("error_dz", Float32, self.error_dz)
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_remap)
        # rospy.Subscriber("roll", Float32, self.error_roll)
        # rospy.Subscriber("pitch", Float32, self.error_pitch)
        rospy.Subscriber("reinit_pose", Float32, self.reinit_pose)
        rospy.Subscriber("mavros/local_position/velocity", TwistStamped, self.lpos_velocity_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.lpos_pose_cb)

        self.pub_lpe = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.pub_spt = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_cmd_vel_attitude = rospy.Publisher('mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.pub_att_thr = rospy.Publisher('mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.publish_cmd_vel)
        self.odom = rospy.Publisher('odom', Odometry, queue_size=10)

        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # self.move_base.wait_for_server(rospy.Duration(60))

        self.rate = rospy.Rate(50) # 20hz
        self.has_global_pos = True
        self.setpointX = 0.0
        self.transUpdated = False
        self.roll = 0.00
        self.pitch = 0.00
        self.yaw = 0.00
        self.rotm = None
        self.vz = 0.0
        self.vy = 0.0
        self.vx = 0.0
        self.thr = 0.6
        self.hover_z = 1.0
        self.last_twist_updated = rospy.Time.from_sec(0.0)
        self.hover_x = 0.0
        self.hover_y = 0.0


        while not rospy.is_shutdown():
            # print 'roll', euler[0], '\t pitch', euler[1]
            # self.pub_rplidar_tf.sendTransform((0,0,0), quaternion_from_euler(0,0,np.pi), rospy.Time.now(), "rplidar_link", "base_link")
            try:
                (self.trans, self.rot) = self.sub_robot_tf.lookupTransform('/map', '/base_link', rospy.Time(0))
                # (self.trans, self.rot) = self.sub_robot_tf.lookupTransform('/odom', '/base_link', rospy.Time(0))
                self.transUpdated = True
                # pose = self.pose_from_tf(self.trans, self.rot)
                # print pose
            except:
                # print 'err'
                self.transUpdated = False
            self.rate.sleep()
            # self.publish_spt()
            self.lpe(self.errorDx)

            # self.last_twist.twist.linear.x = 0.1
            diff = rospy.Time.now() - self.last_twist_updated
            # print 'time diff: ', diff.secs
            # self.pub_cmd_vel.publish(self.last_twist)
            if (diff.secs <= 1):
                # print 'cmd_vel: true'
                self.hover_x = self.lpos.x
                self.hover_y = self.lpos.y
                self.pub_cmd_vel.publish(self.last_twist)
            else:
                print 'cmd_vel: false, hovering in place (x,y): ', self.hover_x, self.hover_y
                self.publish_spt(x=self.hover_x, y=self.hover_y)
            # self.last_twist_updated = False
            # self.pub_cmd_vel_attitude.publish(self.last_twist)

            # if (self.z < 0.8*self.hover_z):
            #     self.pub_att_thr.publish(self.thr + 0.05)
            # else:
            #     self.pub_att_thr.publish(self.thr - 0.08*self.vz)

            # for position setpoint controller
            # self.publish_spt()

            # testing cmd_vel velocity setpoint controller
            # self.publish_cmd_vel()

            # self.lpe(self.errorDx) #for using lsq errorDx = 0
            # self.lpe(self.fakeX) #for using fakeX toggled by RCIn[7]

    #
    # General callback functions used in tests
    #

    def publish_spt(self, x=0, y=0, z=1):
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z

        # For demo purposes we will lock yaw/heading to north.
        quaternion = quaternion_from_euler(np.pi, 0, np.pi/2)
        pos.pose.orientation = Quaternion(*quaternion)
        pos.header.stamp = rospy.Time.now()
        self.pub_spt.publish(pos)

    # def publish_moveBaseGoal(self):
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     goal.target.pose.position.x = 0.0
    #     goal.target.pose.position.y = 0.0
    #     goal.target.pose.orientation.w = 1.0
    #     self.move_base.send_goal(goal)

    def lpos_velocity_cb(self, data):
        # earth-fixed ENU
        self.vz = data.twist.linear.z
        self.vx = data.twist.linear.x
        self.vy = data.twist.linear.y

    def lpos_pose_cb(self, data):
        # earth-fixed ENU
        self.lpos = data.pose.position

    def publish_cmd_vel(self, data):
        # subscribed data is in base-link ENU
        # rotm = euler_matrix(0, 0, self.yaw , 'sxyz')
        # trans = np.array([data.linear.x, data.linear.y, data.linear.z])
        rotm = euler_matrix(0, 0, self.yaw , 'sxyz')
        trans = np.array([data.linear.x, data.linear.y, data.linear.z])
        # print 'before: ', trans
        trans = np.dot(rotm[0:3,0:3], trans)
        # print 'after: ', trans
        data.linear.x = trans[0]
        data.linear.y = trans[1]
        # published cmd_vel is in earth-fixed ENU
        twist = TwistStamped()
        twist.header = Header()
        twist.header.frame_id = "odom"

        if (np.absolute(data.linear.x) >= 0.0001 or np.absolute(data.linear.y) >= 0.0001):
            twist.twist.linear.x = data.linear.x
            twist.twist.linear.y = data.linear.y
            twist.twist.linear.z = 0
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            # print data.angular.z
            twist.twist.angular.z = data.angular.z

            # twist.header.stamp = rospy.Time.now()
            # self.pub_cmd_vel.publish(twist)
            self.last_twist = twist
            # self.pub_cmd_vel_attitude.publish(twist)
        elif (np.absolute(data.angular.z) > 0.0001):
            # try to hold position while yawing
            twist.twist.linear.x = -0.1*self.vx
            twist.twist.linear.y = -0.1*self.vy
            twist.twist.linear.z = 0
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = data.angular.z
            self.last_twist = twist
        else:
            # no cmd_vel
            twist.twist.linear.x = 0
            twist.twist.linear.y = 0
            twist.twist.linear.z = 0
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = 0
            self.last_twist = twist
        self.last_twist.header.stamp = rospy.Time.now()
        self.last_twist_updated = rospy.Time.now()

    def publish_odom(self, x , y):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom_quat = quaternion_from_euler(0, 0, 0)
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        rotm = euler_matrix(0, 0, self.yaw , 'sxyz')
        trans = np.array([self.vx, self.vy, self.vz])
        # print 'before: ', trans
        trans = np.dot(rotm[0:3,0:3], trans)
        # print 'after: ', trans
        odom.twist.twist.linear.x = trans[0]
        odom.twist.twist.linear.y = trans[1]

        odom.child_frame_id = "base_link"
        self.odom.publish(odom)

    def lpe(self, errorDx):
        if (self.error_updated[0] == True and self.error_updated[1] == True and self.transUpdated == True):

            # self.error_updated[0] = False
            # self.error_updated[1] = False
            pos = PoseStamped()
            pos.header = Header()
            pos.header.frame_id = "base_link"
            ## pixhawk is using NED convention i.e. x (front/north), y (right,east), z(down)
            ## however lsq x-y-z is left, front, down
            #pos.pose.position.x = errorDx
            #pos.pose.position.y = self.errorDy
            #pos.pose.position.z = self.z

            # working conversion for gazebostates, must be sent in ENU
            # pos.pose.position.y = self.errorDy
            # pos.pose.position.x = self.errorDx
            q = self.local_position.orientation
            euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            # print 'roll ', euler[0], '\t pitch ', euler[1], '\t yaw ', euler[2]
            # q = quaternion_from_euler(np.pi+euler[0], euler[1], np.pi/2+euler[2]) #x,y,z, 'zyx order'
            # print 'yaw: ', self.yaw

            self.yaw = -(-euler[2]+np.pi/2)
            q = quaternion_from_euler(np.pi+euler[0], euler[1], np.pi/2-np.pi/2+euler[2]) #x,y,z, 'zyx order'
            # q = quaternion_from_euler(np.pi+euler[0], euler[1], np.pi/2-np.pi/2+euler[2]) #x,y,z, 'zyx order' when facing east
            pos.pose.orientation = Quaternion(*q) # quaternion must be sent in NED (weird MAVROS implementation)

            # working conversion for carto
            # pos.pose.position.y = self.errorDx
            # pos.pose.position.x = -self.errorDy

            # self.rotm = euler_matrix(0, 0, euler[2]-np.pi/2, 'sxyz')
            self.rotm = euler_matrix(0, 0, 0, 'sxyz')
            trans = np.dot(self.rotm[0:3,0:3], self.trans)
            self.errorDy = trans[1] # rotate from cartoY in the body frame to earth-fixed frame
            self.errorDx = trans[0] # cartoX in the body frame

            self.transUpdated = False
            # trial conversion for carto
            pos.pose.position.y = self.errorDy
            pos.pose.position.x = self.errorDx

            # q = quaternion_from_euler(self.roll, self.pitch, np.pi/4+self.yaw) #x,y,z, 'zyx order'
            # pos.pose.orientation.x = q[0]
            # pos.pose.orientation.y = q[1]
            # pos.pose.orientation.z = q[2]
            # pos.pose.orientation.w = q[3]

            # pos.pose.position.x = pos.pose.position.x - self.fakeY
            pos.pose.position.z = self.z
            # print 'x ', pos.pose.position.x, '\t y ', pos.pose.position.y, '\t z ', pos.pose.position.z

            # For demo purposes we will lock yaw/heading to north.

            # print q
            # euler[2] = self.errorDz
            # print q

            #q = quaternion_from_euler(0, 0, -self.errorDz+np.pi/2)
            #self.errorDz = np.pi/4

            #pos.pose.orientation.x = q[0]
            #pos.pose.orientation.y = q[1]
            #pos.pose.orientation.z = q[3]
            #pos.pose.orientation.w = q[2]

            # print q
            #pos.pose.orientation = self.local_position.orientation
            #q = pos.pose.orientation

            #euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            #print 'roll ', np.pi+euler[0], '\t pitch ', euler[1], '\t yaw ', np.pi/2+euler[2]

            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_lpe.publish(pos)
            self.publish_odom(self.trans[0], self.trans[1])

    def pose_from_tf(self, trans, rot):
        euler = np.array(euler_from_quaternion((rot[0], rot[1], rot[2], rot[3])))
        rotm = euler_matrix(euler[0], euler[1], euler[2], 'sxyz')
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]
        pose = np.dot(rotm[0:3,0:3], trans)
        # print 'roll ', euler[0], '\t pitch ', euler[1], '\t yaw ', np.pi/4+euler[2]
        return pose

    def reinit_pose(self, msg):
        if msg.data >= 1:
            self.spX = self.trans[0]
            self.spY = self.trans[1]

    def position_callback(self, data):
        self.local_position = data
        # self.z = 0
        # self.lpe()

    def imu_remap(self, data):

        if (self.imuCalibrated == False and self.imuOffset[0] <= 50):
            self.imuOffset[1] += data.angular_velocity.x
            self.imuOffset[2] += data.angular_velocity.y
            self.imuOffset[3] += data.angular_velocity.z
            self.imuOffset[4] += data.linear_acceleration.x
            self.imuOffset[5] += data.linear_acceleration.y
            self.imuOffset[0] += 1
            print 'calibrating imu'
            if self.imuOffset[0] == 50:
                for i in range(5):
                    self.imuOffset[i+1] /= 50
                self.imuCalibrated = True
                print 'calibrated'

        if (self.imuCalibrated):
            imu = data
            imu.header = Header()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "imu_link"
            wx = imu.angular_velocity.x #- self.imuOffset[1]
            wy = imu.angular_velocity.y #- self.imuOffset[2]
            wz = imu.angular_velocity.z #- self.imuOffset[3]
            ax = imu.linear_acceleration.x - self.imuOffset[4]
            ay = imu.linear_acceleration.y - self.imuOffset[5]
            az = imu.linear_acceleration.z
            self.imu = imu
            self.imu.linear_acceleration.x = -ay #-(ay) #-ay #direction of the ENU +ve y-axis
            self.imu.linear_acceleration.y = -ax # -ax # ax #-ax #direction of the ENU -ve x-axis
            self.imu.linear_acceleration.z = az
            self.imu.angular_velocity.y = wx
            self.imu.angular_velocity.x = -wy
            self.imu.angular_velocity.z = wz
            q = self.imu.orientation
            euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
            self.pub_imu_tf.sendTransform((0,0,0), quaternion_from_euler(euler[1],-euler[0],0), rospy.Time.now(), "imu_link", "base_link")
            # self.imu.orientation.x = 0
            # self.imu.orientation.y = 0
            # self.imu.orientation.z = 0
            # self.imu.orientation.w = 1
            self.pub_imu.publish(self.imu)

    def setpoint_callback(self, data):
        self.spX = data.position.y #because of ned to enu shit
        self.spY = -data.position.x

    def global_position_callback(self, data):
        self.has_global_pos = True

    def range_callback(self, msg):
        self.z = msg.ranges[0]
        if self.z >= 14:
            self.z = 0

    def error_roll(self, msg):
        self.roll = msg.data

    def error_pitch(self, msg):
        self.pitch = msg.data

    def gazebo_pose(self, msg):
        # print msg.pose[2].position.x
        name = msg.name
        index = name.index('iris_rplidar')
        self.z = msg.pose[index].position.z
        self.errorDy = msg.pose[index].position.y
        self.errorDx = msg.pose[index].position.x
        self.local_position = msg.pose[index]

        self.error_updated[0] = True
        self.error_updated[1] = True

    def error_dx(self, msg):
        self.errorDx = msg.data
        self.error_updated[0] = True

    def error_dy(self, msg):
        self.errorDy = msg.data
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
