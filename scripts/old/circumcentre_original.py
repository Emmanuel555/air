#!/usr/bin/python

import rospy
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from rospy.numpy_msg import numpy_msg

# simple class to contain the node's variables and code

class CentreFinder:     # class constructor; subscribe to topics and advertise intent to publish

    def __init__(self, v1 = np.ndarray([0,0]), v2 = np.ndarray([0,0]), v3 = np.ndarray([0,0]) ):

        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.updated = [False, False, False]
        self.orient = [3*np.pi/4, np.pi/4, -np.pi/4, -3*np.pi/4]
        self.update_rate = 10


        rospy.Subscriber("teraranger0", Range, self.callback1)
        rospy.Subscriber("teraranger1", Range, self.callback2)
        rospy.Subscriber("teraranger2", Range, self.callback3)

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=10)
	self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=10)

        rate = rospy.Rate(self.update_rate)	

        while not rospy.is_shutdown():

		if (self.updated[0]==True and self.updated[1]==True and self.updated[2]==True):
			self.error()

		rate.sleep()

    def callback1(self, msg):
        v = msg.range
        self.v1 = self.rotate(v, self.orient[1])
        self.updated[0] = True

    def callback2(self, msg):
        v = msg.range
        self.v2 = self.rotate(v, self.orient[2])
        self.updated[1] = True

    def callback3(self, msg):
        v = msg.range
        self.v3 = self.rotate(v, self.orient[3])
        self.updated[2] = True

    def rotate(self, r, angle):
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        return np.array([x,y])

    def error(self):
        A = np.array([[1, 2*self.v1[0], 2*self.v1[1]],
        [1, 2*self.v2[0], 2*self.v2[1]],
        [1, 2*self.v3[0], 2*self.v3[1]]])

        B = np.array([[self.v1[0]**2 + self.v1[1]**2],
        [self.v2[0]**2 + self.v2[1]**2],
        [self.v3[0]**2 + self.v3[1]**2]])

        At = A.transpose()

        x = np.dot(np.linalg.inv(np.dot(At, A)), np.dot(At, B))

        dx = x[1]
        dy = x[2]

        self.updated[0] = False
        self.updated[1] = False
        self.updated[2] = False

        # a = np.array([dx, dy], dtype=np.float32)
        a = [dx, dy]
        # print rospy.get_name(), a

        self.errorDx_pub.publish(dx)
	self.errorDy_pub.publish(dy)

if __name__ == "__main__":

    rospy.init_node("centre_finder_node")
    node = CentreFinder()

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
