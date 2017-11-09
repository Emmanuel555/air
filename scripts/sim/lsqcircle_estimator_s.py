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
        self.sensorCount = 6
        self.debug = debug
        self.roll = 0
        self.pitch = 0
        self.M = np.array([[87, 131, 70, 112.5],
        [58.5, 94.5, 51, 84],
        [60.5, 92.5, 52, 84.5],
        [94, 60.5, 86, 53.5],
        [91, 57.5, 89.5, 56.5],
        [141.5, 89.5, 134, 83]])
        self.M = self.M/100;

        self.updated = [False, False, False, False, False, False]
        self.orient = [-np.pi/4, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2,  np.pi/4]
        self.offset = np.array([[0.2256, -0.1741, 0],
        [0.1739, -0.1915, 0],
        [-0.1739, -0.1915, 0],
        [-0.1739, 0.1915, 0],
        [0.1739, 0.1915, 0],
        [0.2256, 0.1741, 0]])

        self.update_rate = 10

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
            # self.lsqline_pub()
            self.lsqcircle_pub_i()
            # self.lsqcircle_pub()
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
        # self.roll = 0 #offset of 1 deg
        # self.pitch = 0
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
            self.updatePolygonVertex_old(v, i)

    def updatePolygonVertex_old(self, msg, index, debug=False):
        # v = msg
        v = msg.ranges[0]
        v_min = 200.0/1000.0
        v_max = 14.0
        valid = True
        # print index, ": ", v
        if ((v <= v_min) or (v >= v_max)):
            # print "False: ", index
            valid = False
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

    def lsqcircle_pub_i(self, debug = True):
        rotm = euler_matrix(self.roll, self.pitch, 0, 'sxyz')
        #rotm = euler_matrix(np.pi/6, 0, 0, 'sxyz')
        A = self.bodyXYZ[0:3, 0:2]
        updated = self.updated # lock the current updated matrix
        vs = np.array([[self.v0[0], self.v0[1], 0], # lock in all the vertices
        [self.v1[0], self.v1[1], 0],
        [self.v2[0], self.v2[1], 0],
        [self.v3[0], self.v3[1], 0],
        [self.v4[0], self.v4[1], 0],
        [self.v5[0], self.v5[1], 0]])
        trues = np.sum(updated)
        Alsq = np.zeros((trues,3))
        Blsq = np.zeros((trues,1))
        if (debug):
            print 'updated: ', self.updated
            print 'trues: ', trues
        if (debug):
            print 'A', A
            print 'rotm', rotm
        # print self.v0
        array_index = 0

        for index in range(self.sensorCount):
            B = vs[index, :]
            v = self.projectSubspace(A,B)
            v = np.dot(rotm[0:3,0:3], v)
            if (updated[index]):
                Alsq[array_index, :] = [2*v[0], 2*v[1], 1]
                Blsq[array_index, :] = [v[0]**2+v[1]**2]
                array_index += 1
            if (debug):
                print 'v', 'index', ': ', v
                print "A: ", Alsq
                print "B: ", Blsq

        # for index in range(self.sensorCount):
        #     if index == 0:
        #         B = np.array([self.v0[0], self.v0[1], 0])
        #         v0 = self.projectSubspace(A, B)
        #         v0 = np.dot(rotm[0:3,0:3], v0)
        #         if (updated[index]):
        #             Alsq[array_index, :] = [2*v0[0], 2*v0[1], 1]
        #             Blsq[array_index, :] = [v0[0]**2+v0[1]**2]
        #             array_index += 1
        #         if (debug):
        #             print 'v0', v0
        #             print "A: ", Alsq
        #             print "B: ", Blsq
        #
        #     elif index == 1:
        #         B = np.array([self.v1[0], self.v1[1], 0])
        #         v1 = self.projectSubspace(A, B)
        #         v1 = np.dot(rotm[0:3,0:3], v1)
        #         if (updated[index]):
        #             Alsq[array_index, :] = [2*v1[0], 2*v1[1], 1]
        #             Blsq[array_index, :] = [v1[0]**2+v1[1]**2]
        #             array_index += 1
        #         if (debug):
        #             print 'v1', v1
        #             print "A: ", Alsq
        #             print "B: ", Blsq
        #
        #     elif index == 2:
        #         B = np.array([self.v2[0], self.v2[1], 0])
        #         v2 = self.projectSubspace(A, B)
        #         v2 = np.dot(rotm[0:3,0:3], v2)
        #         if (updated[index]):
        #             Alsq[array_index, :] = [2*v2[0], 2*v2[1], 1]
        #             Blsq[array_index, :] = [v2[0]**2+v2[1]**2]
        #             array_index += 1
        #         if (debug):
        #             print 'v2', v2
        #             print "A: ", Alsq
        #             print "B: ", Blsq
        #
        #     elif index == 3:
        #         B = np.array([self.v3[0], self.v3[1], 0])
        #         v3 = self.projectSubspace(A, B)
        #         v3 = np.dot(rotm[0:3,0:3], v3)
        #         if (updated[index]):
        #             Alsq[array_index, :] = [2*v3[0], 2*v3[1], 1]
        #             Blsq[array_index, :] = [v3[0]**2+v3[1]**2]
        #             array_index += 1
        #         if (debug):
        #             print 'v3', v3
        #             print "A: ", Alsq
        #             print "B: ", Blsq
        #
        #     elif index == 4:
        #         B = np.array([self.v4[0], self.v4[1], 0])
        #         v4 = self.projectSubspace(A, B)
        #         v4 = np.dot(rotm[0:3,0:3], v4)
        #         if (updated[index]):
        #             Alsq[array_index, :] = [2*v4[0], 2*v4[1], 1]
        #             Blsq[array_index, :] = [v4[0]**2+v4[1]**2]
        #             array_index += 1
        #         if (debug):
        #             print 'v4', v4
        #             print "A: ", Alsq
        #             print "B: ", Blsq
        #
        #     elif index == 5:
        #         B = np.array([self.v5[0], self.v5[1], 0])
        #         v5 = self.projectSubspace(A, B)
        #         v5 = np.dot(rotm[0:3,0:3], v5)
        #         if (updated[index]):
        #             Alsq[array_index, :] = [2*v5[0], 2*v5[1], 1]
        #             Blsq[array_index, :] = [v5[0]**2+v5[1]**2]
        #             array_index += 1
        #         if (debug):
        #             print 'v5', v5
        #             print "A: ", Alsq
        #             print "B: ", Blsq


        # A = np.array([[2*v0[0], 2*v0[1], 1],
        # [2*v1[0], 2*v1[1], 1],
        # [2*v2[0], 2*v2[1], 1],
        # [2*v3[0], 2*v3[1], 1],
        # [2*v4[0], 2*v4[1], 1],
        # [2*v5[0], 2*v5[1], 1]])
        #
        # B = np.array([[v0[0]**2+v0[1]**2],
        # [v1[0]**2+v1[1]**2],
        # [v2[0]**2+v2[1]**2],
        # [v3[0]**2+v3[1]**2],
        # [v4[0]**2+v4[1]**2],
        # [v5[0]**2+v5[1]**2]])

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

        if (np.sum(Alsq) != 0 and np.sum(Blsq) != 0):
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
            # print 'A: \t', A
            # print 'B: \t', B
            # print 'x: \t', x

        self.errorDx_pub.publish(dx)
        self.errorDy_pub.publish(dy)
        self.errorDz_pub.publish(alpha)


    def lsqcircle_pub(self, debug = False):
        rotm = euler_matrix(self.roll, self.pitch, 0, 'sxyz')
        #rotm = euler_matrix(np.pi/6, 0, 0, 'sxyz')
        A = self.bodyXYZ[0:3, 0:2]

        #if (debug):
            #print 'A', A
            #print 'rotm', rotm
        B = np.array([self.v0[0], self.v0[1], 0])
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

        A = np.array([[2*v0[0], 2*v0[1], 1],
        [2*v1[0], 2*v1[1], 1],
        [2*v2[0], 2*v2[1], 1],
        [2*v3[0], 2*v3[1], 1],
        [2*v4[0], 2*v4[1], 1],
        [2*v5[0], 2*v5[1], 1]])

        B = np.array([[v0[0]**2+v0[1]**2],
        [v1[0]**2+v1[1]**2],
        [v2[0]**2+v2[1]**2],
        [v3[0]**2+v3[1]**2],
        [v4[0]**2+v4[1]**2],
        [v5[0]**2+v5[1]**2]])

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

        dx = x[0]
        dy = x[1]
        r  = np.sqrt(x[2]+dx**2+dy**2)
        alpha = 0

        if self.debug or debug:
            print 'dX: \t', dx
            print 'dY: \t', dy
            print 'r: \t', r
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
