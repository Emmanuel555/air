#!/usr/bin/python

import rospy
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Range, LaserScan
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
from teraranger_array.msg import RangeArray

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
        self.M = np.array([[120.5, 70, 121.1, 70.5],
        [94.5, 60, 85.9, 50],
        [91, 51.5, 85.9, 50],
        [42, 81, 50, 85.9],
        [51.5, 90, 50, 85.9],
        [68.5, 117.5, 70.5, 121.1]])
        self.M = self.M/100

        self.updated = [False, False, False, False, False, False]
        self.orient = [-np.pi/4, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2,  np.pi/4]
        self.offset = np.array([[0.2256, -0.1741, 0],
        [0.1739, -0.1915, 0],
        [-0.1739, -0.1915, 0],
        [-0.1739, 0.1915, 0],
        [0.1739, 0.1915, 0],
        [0.2256, 0.1741, 0]])

        self.update_rate = 20

        self.bodyXYZ = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        # rospy.Subscriber("teraranger_hub_one", RangeArray, self.updatePolygonVertex, queue_size=1)
        rospy.Subscriber("teraranger1/laser/scan", LaserScan, self.updatePolygonVertex_old, 0)
        rospy.Subscriber("teraranger2/laser/scan", LaserScan, self.updatePolygonVertex_old, 1)
        rospy.Subscriber("teraranger3/laser/scan", LaserScan, self.updatePolygonVertex_old, 2)
        rospy.Subscriber("teraranger4/laser/scan", LaserScan, self.updatePolygonVertex_old, 3)
        rospy.Subscriber("teraranger5/laser/scan", LaserScan, self.updatePolygonVertex_old, 4)
        rospy.Subscriber("teraranger6/laser/scan", LaserScan, self.updatePolygonVertex_old, 5)

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=1)
        self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=1)
        self.errorDz_pub = rospy.Publisher("error_dz", Float32, queue_size=1)
        self.errorDr_pub = rospy.Publisher("roll", Float32, queue_size=1)
        self.errorDp_pub = rospy.Publisher("pitch", Float32, queue_size=1)

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.updateRPY)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            # if (self.updated[0]==True and self.updated[1]==True and self.updated[2]==True and self.updated[3]==True):
            self.bodyXYZ = self.bodyRotation(-self.pitch, -self.roll) #update the body rotation matrix
            #self.bodyXYZ = self.bodyRotation(-0, -np.pi/6)
            self.lsqline_pub()
            rate.sleep()

    def sensorComp(self, old, i):
        M = self.M
        A = np.array([[M[i,0], 1],[M[i,1], 1]])
        Y = np.array([M[i,2], M[i,3]])
        X = np.dot(np.linalg.inv(A),Y)
        new = X[0]*old + X[1]
        return new

    def bodyRotation(self, pitch, roll, debug=False):
        bodyXYZ = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0], [0, 0, 0, 1]])
        rotmZ = euler_matrix(0,0,0,'sxyz')
        bodyX1Y1Z1 = np.dot(rotmZ, bodyXYZ)
        # bodyX1Y1Z1 = rotmZ * bodyXYZ
        rotmY = euler_matrix(0,pitch,0,'sxyz') #-pitch because px4 y-axis points to the left.
        bodyX2Y2Z2 = np.dot(rotmY, bodyX1Y1Z1)
        # bodyX2Y2Z2 = rotmY * bodyX1Y1Z1
        rotmX = euler_matrix(roll,0,0,'sxyz')
        bodyX3Y3Z3 = np.dot(rotmX, bodyX2Y2Z2)
        # bodyX3Y3Z3 = rotmX * bodyX2Y2Z2

        if debug or self.debug:
            print 'bodyX1Y1Z1 ', bodyX1Y1Z1
            print 'bodyX2Y2Z2 ', bodyX2Y2Z2
            print 'bodyX3Y3Z3 ', bodyX3Y3Z3
            print 'rotmZ ', rotmZ
            print 'rotmY ', rotmY
            print 'rotmX ', rotmX

        return bodyX3Y3Z3

    def updateRPY(self, data, debug=False):
        local_position = data
        q = local_position.pose.orientation
        euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
        self.roll = euler[0] #offset of 1 deg
        self.pitch = euler[1]
        self.errorDr_pub.publish(self.roll)
        self.errorDp_pub.publish(self.pitch)

        if debug or self.debug:
            print 'roll ', self.roll, '\t pitch ', self.pitch, '\t yaw ', -(euler[2]-np.pi/2)

    def updatePolygonVertex(self, msg, debug=False):
        ranges = msg.ranges
        sensorCount = 6
        for i in range(sensorCount):
            v = ranges[i].range
            if (debug):
                print 'teraranger' , i, 'distance ', v
            v = self.sensorComp(v,i)
            self.updatePolygonVertex_old(v, i)

    def updatePolygonVertex_old(self, msg, index, debug=False):
        # v = msg
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

    def projectSubspace(self, A, B, debug=False):
        x = np.linalg.lstsq(A,B)[0]
        v = np.dot(A, x)
        if (debug):
            print 'A: ', A
            print 'B: ', B
            print 'x: ', x
            print 'v: ', v
        return v

    def lsqline_pub(self, debug = False):
        rotm = euler_matrix(self.roll, self.pitch, 0, 'sxyz')
        #rotm = euler_matrix(np.pi/6, 0, 0, 'sxyz')
        A = self.bodyXYZ[0:3, 0:2]

        #if (debug):
            #print 'A', A
            #print 'rotm', rotm

        B = np.array([self.v0[0], self.v0[1], 0])
        #if (debug):
            #print 'B', B

        v0 = self.projectSubspace(A, B)
        v0 = np.dot(rotm[0:3,0:3], v0)
        if (debug):
            print 'v0', v0

        B = np.array([self.v1[0], self.v1[1], 0])
        v1 = self.projectSubspace(A, B)
        v1 = np.dot(rotm[0:3,0:3], v1)
        if (debug):
            print 'v1', v1

        B = np.array([self.v2[0], self.v2[1], 0])
        v2 = self.projectSubspace(A, B)
        v2 = np.dot(rotm[0:3,0:3], v2)
        if (debug):
            print 'v2', v2

        B = np.array([self.v3[0], self.v3[1], 0])
        v3 = self.projectSubspace(A, B)
        v3 = np.dot(rotm[0:3,0:3], v3)
        if (debug):
            print 'v3', v3

        B = np.array([self.v4[0], self.v4[1], 0])
        v4 = self.projectSubspace(A, B)
        v4 = np.dot(rotm[0:3,0:3], v4)
        if (debug):
            print 'v4', v4

        B = np.array([self.v5[0], self.v5[1], 0])
        v5 = self.projectSubspace(A, B)
        v5 = np.dot(rotm[0:3,0:3], v5)
        if (debug):
            print 'v5', v5

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
        dx = 0

        if self.debug or debug:
            print 'rL: \t', rL
            print 'rR: \t', rR
            print 'yaw: \t', alpha
            print 'centre: \t', dy
            #print 'A: \t', A
            #print 'B: \t', B
            print 'x: \t', x

        self.errorDx_pub.publish(dx)
        self.errorDy_pub.publish(dy)
        self.errorDz_pub.publish(alpha)


if __name__ == "__main__":

    rospy.init_node("centroid_finder_node")
    node = CentroidFinder()

    #rospy.spin()

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
