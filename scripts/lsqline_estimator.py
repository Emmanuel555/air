#!/usr/bin/python

import rospy
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Range, LaserScan
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix

# simple class to contain the node's variables and code
class CentroidFinder:     # class constructor; subscribe to topics and advertise intent to publish

    def __init__(self, v0 = np.array([1,1]), v1 = np.array([-1,-1]), v2 = np.array([1,-1]), v3 = np.array([-1,1]), v4 = np.array([1,0]), v5 = np.array([-1,0]), debug=False ):

        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.v4 = v3
        self.v5 = v3
        self.debug = debug
        self.roll = 0
        self.pitch = 0

        self.updated = [False, False, False, False, False, False]
        self.orient = [-np.pi/4, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2,  np.pi/4]
        self.offset = np.array([[0.2256, -0.1741, 0],
        [0.1739, -0.1915, 0],
        [-0.1739, -0.1915, 0],
        [-0.1739, 0.1915, 0],
        [0.1739, 0.1915, 0],
        [0.2256, 0.1741, 0]])

        self.update_rate = 100

        self.bodyXYZ = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        rospy.Subscriber("teraranger1/laser/scan", LaserScan, self.updatePolygonVertex, 0)
        rospy.Subscriber("teraranger2/laser/scan", LaserScan, self.updatePolygonVertex, 1)
        rospy.Subscriber("teraranger3/laser/scan", LaserScan, self.updatePolygonVertex, 2)
        rospy.Subscriber("teraranger4/laser/scan", LaserScan, self.updatePolygonVertex, 3)
        rospy.Subscriber("teraranger5/laser/scan", LaserScan, self.updatePolygonVertex, 4)
        rospy.Subscriber("teraranger6/laser/scan", LaserScan, self.updatePolygonVertex, 5)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.updateRPY)

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=10)
        self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=10)
        self.errorDz_pub = rospy.Publisher("error_dz", Float32, queue_size=10)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            # if (self.updated[0]==True and self.updated[1]==True and self.updated[2]==True and self.updated[3]==True):
            self.bodyXYZ = self.bodyRotation(self.pitch, self.roll) #update the body rotation matrix
            self.lsqline_pub()
            rate.sleep()

    def bodyRotation(self, pitch, roll, debug =False):
        bodyXYZ = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0], [0, 0, 0, 1]])
        rotmZ = euler_matrix(0,0,0,'sxyz')
        bodyX1Y1Z1 = np.dot(rotmZ, bodyXYZ)
        # bodyX1Y1Z1 = rotmZ * bodyXYZ
        rotmY = euler_matrix(0,-np.pi/4,0,'sxyz')
        bodyX2Y2Z2 = np.dot(rotmY, bodyX1Y1Z1)
        # bodyX2Y2Z2 = rotmY * bodyX1Y1Z1
        rotmX = euler_matrix(-np.pi/4,0,0,'sxyz')
        bodyX3Y3Z3 = np.dot(rotmX, bodyX2Y2Z2)
        # bodyX3Y3Z3 = rotmX * bodyX2Y2Z2

        if debug or self.debug:
            print 'bodyX1Y1Z1 ', bodyX1Y1Z1
            print 'bodyX2Y2Z2 ', bodyX2Y2Z2
            print 'bodyX3Y3Z3 ', bodyX3Y3Z3
            # print 'rotmZ ', rotmZ
            # print 'rotmY ', rotmY
            # print 'rotmX ', rotmX

        return bodyX3Y3Z3

    def updateRPY(self, data, debug=False):
        local_position = data
        q = local_position.pose.orientation
        euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
        self.roll = euler[0]
        self.pitch = euler[1]

        if debug or self.debug:
            print 'roll ', self.roll, '\t pitch ', self.pitch, '\t yaw ', euler[2]

    def updatePolygonVertex(self, msg, index, debug=False):
        v = msg.ranges[0]
        v_min = 200/1000
        v_max = 14
        if (v < v_min or v > v_max):
            return False
        if index == 0:
            self.v0 = self.offset[0, 0:2] + self.rotate(v, self.orient[0])
            self.updated[0] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v0
        elif index == 1:
            self.v1 = self.offset[index, 0:2] + self.rotate(v, self.orient[1])
            self.updated[1] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v1
        elif index == 2:
            self.v2 = self.offset[index, 0:2] + self.rotate(v, self.orient[2])
            self.updated[2] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v2
        elif index == 3:
            self.v3 = self.offset[index, 0:2] + self.rotate(v, self.orient[3])
            self.updated[3] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v3
        elif index == 4:
            self.v4 = self.offset[index, 0:2] + self.rotate(v, self.orient[4])
            self.updated[4] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v4
        elif index == 5:
            self.v5 = self.offset[index, 0:2] + self.rotate(v, self.orient[5])
            self.updated[5] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v5

    def rotate(self, r, angle):
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        return np.array([x,y])

    def projectSubspace(self, A, B):
        x = np.linalg.lstsq(A,B)[0]
        v = np.dot(A, x)
        return v

    def lsqline_pub(self, debug = True):

        A = self.bodyXYZ[0:3, 0:3]

        B = np.array([self.v0[0], self.v0[1], 0])
        v0 = self.projectSubspace(A, B)

        B = np.array([self.v1[0], self.v1[1], 0])
        v1 = self.projectSubspace(A, B)

        B = np.array([self.v2[0], self.v2[1], 0])
        v2 = self.projectSubspace(A, B)

        B = np.array([self.v3[0], self.v3[1], 0])
        v3 = self.projectSubspace(A, B)

        B = np.array([self.v4[0], self.v4[1], 0])
        v4 = self.projectSubspace(A, B)

        B = np.array([self.v5[0], self.v5[1], 0])
        v5 = self.projectSubspace(A, B)

        A = np.array([[v0[0], -1, 0],
        [v1[0], -1, 0],
        [v2[0], -1, 0],
        [v3[0], 0, -1],
        [v4[0], 0, -1],
        [v5[0], 0, -1]])

        B = np.array([[-v0[1]],
        [-v1[1]],
        [-v2[1]],
        [-v3[1]],
        [-v4[1]],
        [-v5[1]]])

        # A = np.array([[self.v0[0], -1, 0],
        # [self.v1[0], -1, 0],
        # [self.v2[0], -1, 0],
        # [self.v3[0], 0, -1],
        # [self.v4[0], 0, -1],
        # [self.v5[0], 0, -1]])
        #
        # B = np.array([[-self.v0[1]],
        # [-self.v1[1]],
        # [-self.v2[1]],
        # [-self.v3[1]],
        # [-self.v4[1]],
        # [-self.v5[1]]])

        self.updated[0] = False
        self.updated[1] = False
        self.updated[2] = False
        self.updated[3] = False
        self.updated[4] = False
        self.updated[5] = False

        # At = A.transpose()
        #
        # x = np.dot(np.linalg.inv(np.dot(At, A)), np.dot(At, B))

        x = np.linalg.lstsq(A,B)[0];

        alpha = np.arctan(x[0])
        rR = x[1] * np.cos(alpha)
        rL = x[2] * np.cos(alpha)

        width = abs(rL) + abs(rR)
        dy = (width/2) - rL
        dx = -0.05

        if self.debug or debug:
            print 'rL: \t', rL
            print 'rR: \t', rR
            print 'yaw: \t', alpha
            print 'centre: \t', dy
            # print 'A: \t', A
            # print 'B: \t', B
            # print 'x: \t', x

        self.errorDx_pub.publish(dx)
        self.errorDy_pub.publish(dy)
        self.errorDz_pub.publish(alpha)


if __name__ == "__main__":

    rospy.init_node("centroid_finder_node")
    node = CentroidFinder()

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
