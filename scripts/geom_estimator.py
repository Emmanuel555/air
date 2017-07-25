#!/usr/bin/python

import rospy
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Range, LaserScan
from rospy.numpy_msg import numpy_msg

# simple class to contain the node's variables and code
class GeomEstimator:     # class constructor; subscribe to topics and advertise intent to publish

    def __init__(self, v0 = 1, v1 = 1, v2 = 1, v3 = 1, debug=False ):

        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.debug = debug

        self.R = 2.5 #estimated 2.5m radius tunnel
        self.forwardSpeed = -0.05 #artifically chase a point


        self.v_list = np.array([self.v0, self.v1, self.v2, self.v3])

        self.updated = [False, False, False, False]
        self.orient = [np.pi/4, -3*np.pi/4, -np.pi/4, 3*np.pi/4]
        self.update_rate = 100

        rospy.Subscriber("teraranger1/laser/scan", LaserScan, self.updateVertex, 0)
        rospy.Subscriber("teraranger2/laser/scan", LaserScan, self.updateVertex, 1)
        rospy.Subscriber("teraranger3/laser/scan", LaserScan, self.updateVertex, 2)
        rospy.Subscriber("teraranger4/laser/scan", LaserScan, self.updateVertex, 3)

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=10)
        self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=10)
        self.errorDz_pub = rospy.Publisher("error_dz", Float32, queue_size=10)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            # if (self.updated[0]==True and self.updated[1]==True and self.updated[2]==True and self.updated[3]==True):
            self.v_list = np.array([self.v0, self.v3, self.v1, self.v2, self.v0])
            self.centroid_pub(self.v_list);
            rate.sleep()

    def updateVertex(self, msg, index):
        v = msg.ranges[0]
        if index == 0:
            self.v0 = v
            self.updated[0] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v0
        elif index == 1:
            self.v1 = v
            self.updated[1] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v1
        elif index == 2:
            self.v2 = v
            self.updated[2] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v2
        elif index == 3:
            self.v3 = v
            self.updated[3] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v3

    def rotate(self, r, angle):
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        return np.array([x,y])

    def solver(self, v):
        A = np.array([[v[0], -1],
            [v[2], 1]])

        B = np.array([[self.R],
            [self.R]])

        x = np.dot(np.linalg.inv(A), B)

        offset = np.pi/4
        alpha = np.arccos(x[0]) - offset
        dx = x[1]

        out = [alpha, dx]

        return out

    def centroid_pub(self, v, debug = False):
        sol = self.solver(v)

        self.updated[0] = False
        self.updated[1] = False
        self.updated[2] = False
        self.updated[3] = False

        if debug == True:
            print 'yaw: \t', sol[0]
            print 'dx: \t', sol[1]

        self.errorDx_pub.publish(sol[1])
        self.errorDy_pub.publish(self.forwardSpeed)
        self.errorDz_pub.publish(sol[0])

if __name__ == "__main__":

    rospy.init_node("geom_estimator_node")
    node = GeomEstimator()

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
