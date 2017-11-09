#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Range

# simple class to contain the node's variables and code

class CentreFinderWrapper:     # class constructor; subscribe to topics and advertise intent to publish

    def __init__(self, v1 = np.ndarray([0,0]), v2 = np.ndarray([0,0]), v3 = np.ndarray([0,0]) ):

        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.updated = [False, False, False]
        self.orient = [np.pi/4, 3*np.pi/4, -3*np.pi/4, -np.pi/4]
        self.update_rate = 10

        rospy.Subscriber("/laser/scan", LaserScan, self.callback)

        self.trone0_pub = rospy.Publisher("teraranger0", Range, queue_size=10)
	self.trone1_pub = rospy.Publisher("teraranger1", Range, queue_size=10)
	self.trone2_pub = rospy.Publisher("teraranger2", Range, queue_size=10)
	self.trone3_pub = rospy.Publisher("teraranger3", Range, queue_size=10)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():

		rate.sleep()

    def callback(self, msg):
        ranges = msg.ranges

	terarangers_msg = Range()
        terarangers_msg.header.frame_id = "base_range"        
        terarangers_msg.radiation_type = 1
        terarangers_msg.field_of_view = 0.0593
        terarangers_msg.min_range = 200
        terarangers_msg.max_range = 14000 # 14 metres     

	v0 = ranges[45] * 1000 #45 degree and convert m to mm
	v1 = ranges[135] * 1000 #135 degree and convert m to mm
	v2 = ranges[225] * 1000 #225 degree and convert m to mm
	v3 = ranges[315] * 1000 #315 degree and convert m to mm
        
	terarangers_msg.header.stamp = rospy.Time.now()
	terarangers_msg.range = v0
	self.trone0_pub.publish(terarangers_msg)
	terarangers_msg.header.stamp = rospy.Time.now()
	terarangers_msg.range = v1
	self.trone1_pub.publish(terarangers_msg)
	terarangers_msg.header.stamp = rospy.Time.now()
	terarangers_msg.range = v2
	self.trone2_pub.publish(terarangers_msg)
	terarangers_msg.header.stamp = rospy.Time.now()
	terarangers_msg.range = v3
	self.trone3_pub.publish(terarangers_msg)

if __name__ == "__main__":

	rospy.init_node("centre_finder_wrapper_node")

	node = CentreFinderWrapper()

	rospy.spin()

    # v1 = 1.5
    # v2 = 2
    # v3 = 0.5
    # node = CentreFinder(v1, v2, v3)
    # node.v1 = node.rotate(node.v1, node.orient[0])
    # print node.v1
    # node.v2 = node.rotate(node.v2, node.orient[1])
    # print node.v2
    # node.v3 = node.rotate(node.v3, node.orient[2])
    # print node.v3
    # print node.error()
