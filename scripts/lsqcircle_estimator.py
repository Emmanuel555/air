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
        self.sensorCount = 6
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
        self.orient = [-np.pi/4, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, np.pi/4]
        self.offset = np.array([[0.2256, -0.1741, 0],
        [0.1739, -0.1915, 0],
        [-0.1739, -0.1915, 0],
        [-0.1739, 0.1915, 0],
        [0.1739, 0.1915, 0],
        [0.2256, 0.1741, 0]])

        self.update_rate = 10

        self.bodyXYZ = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        rospy.Subscriber("teraranger_hub_one", RangeArray, self.updatePolygonVertex, queue_size=1)
        # rospy.Subscriber("teraranger1/laser/scan", LaserScan, self.updatePolygonVertex, 0)
        # rospy.Subscriber("teraranger2/laser/scan", LaserScan, self.updatePolygonVertex, 1)
        # rospy.Subscriber("teraranger3/laser/scan", LaserScan, self.updatePolygonVertex, 2)
        # rospy.Subscriber("teraranger4/laser/scan", LaserScan, self.updatePolygonVertex, 3)
        # rospy.Subscriber("teraranger5/laser/scan", LaserScan, self.updatePolygonVertex, 4)
        # rospy.Subscriber("teraranger6/laser/scan", LaserScan, self.updatePolygonVertex, 5)

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=1)
        self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=1)
        self.errorDz_pub = rospy.Publisher("error_dz", Float32, queue_size=1)
        self.errorDr_pub = rospy.Publisher("roll", Float32, queue_size=1)
        self.errorDp_pub = rospy.Publisher("pitch", Float32, queue_size=1)

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.updateRPY, queue_size=1)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            # if (self.updated[0]==True and self.updated[1]==True and self.updated[2]==True and self.updated[3]==True):
            self.bodyXYZ = self.bodyRotation(-self.pitch, -self.roll) #update the body rotation matrix
            #self.bodyXYZ = self.bodyRotation(-0, -np.pi/6)
            self.lsqcircle_pub()
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
        for i in range(self.sensorCount):
            v = ranges[i].range
            if (debug):
                print 'teraranger' , i, 'distance ', v
            v = self.sensorComp(v,i)
            self.updatePolygonVertex_old([v, ranges[i].range], i)

    def updatePolygonVertex_old(self, msg, index, debug=False):
        v = msg[0]
        v_old = msg[1]
        #print v_old
        v_min = 210.0/1000.0
        v_max = 14.0
        valid = True
        if (v_old < v_min or v_old > v_max): #use the pre-compensated value to check validity
            valid = False
            #print 'False'
        if (index == 0 and valid):
            self.v0 = self.offset[0, 0:2] + self.rotate(v, self.orient[0])
            self.updated[0] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v0
        elif (index == 1 and valid):
            self.v1 = self.offset[index, 0:2] + self.rotate(v, self.orient[1])
            self.updated[1] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v1
        elif (index == 2 and valid):
            self.v2 = self.offset[index, 0:2] + self.rotate(v, self.orient[2])
            self.updated[2] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v2
        elif (index == 3 and valid):
            self.v3 = self.offset[index, 0:2] + self.rotate(v, self.orient[3])
            self.updated[3] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v3
        elif (index == 4 and valid):
            self.v4 = self.offset[index, 0:2] + self.rotate(v, self.orient[4])
            self.updated[4] = True
            if debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v4
        elif (index == 5 and valid):
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

    def lsqcircle_pub(self, debug = False):
        rotm = euler_matrix(self.roll, self.pitch, 0, 'sxyz')
        #rotm = euler_matrix(np.pi/6, 0, 0, 'sxyz')
        A = self.bodyXYZ[0:3, 0:2]
        updated = np.copy(self.updated) # lock the current updated matrix using shallow copy
        vs = np.array([[self.v0[0], self.v0[1], 0], # lock in all the vertices
        [self.v1[0], self.v1[1], 0],
        [self.v2[0], self.v2[1], 0],
        [self.v3[0], self.v3[1], 0],
        [self.v4[0], self.v4[1], 0],
        [self.v5[0], self.v5[1], 0]])
        trues = np.sum(updated)
        Alsq = np.zeros((trues,3))
        Blsq = np.zeros((trues,1))
        array_index = 0
        if (debug):
            print 'updated: ', updated
            print 'trues: ', trues
            # print 'A', A
            # print 'rotm', rotm


        for index in range(self.sensorCount):
            B = vs[index, :]
            v = self.projectSubspace(A,B)
            v = np.dot(rotm[0:3,0:3], v)
            if (debug):
                print 'updated[', index, ']: ', updated[index]
            if (updated[index]):
                Alsq[array_index, :] = [2*v[0], 2*v[1], 1]
                Blsq[array_index, :] = [v[0]**2+v[1]**2]
                array_index += 1
            if (debug):
                print 'v', 'index', ': ', v
                print "A: ", Alsq
                print "B: ", Blsq

        # reset all after consumption
        self.updated[0] = False
        self.updated[1] = False
        self.updated[2] = False
        self.updated[3] = False
        self.updated[4] = False
        self.updated[5] = False

        # At = A.transpose()
        #
        # x = np.dot(np.linalg.inv(np.dot(At, A)), np.dot(At, B))
        # print "A: ", Alsq
        # print "B: ", Blsq

        if (np.sum(Alsq) != 0 and np.sum(Blsq) != 0 and trues >=3): #mandate 3 or more points to publish
            x = np.linalg.lstsq(Alsq,Blsq)[0];
            dx = x[0]
            dy = x[1]
            r  = np.sqrt(x[2]+dx**2+dy**2)
            alpha = 0
        else:
            dx = 0
            dy = 0
            r  = 0
            alpha = 0

        if self.debug or debug:
            print 'dX: \t', dx
            print 'dY: \t', dy
            print 'r: \t', r
            print 'trues: \t', trues
            # print 'A: \t', A
            # print 'B: \t', B
            # print 'x: \t', x

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
