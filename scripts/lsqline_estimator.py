#!/usr/bin/python

import rospy
import numpy as np
import low_pass

from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Range, LaserScan
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
from teraranger_array.msg import RangeArray


# simple class to contain the node's variables and code
class CentroidFinder:     # class constructor; subscribe to topics and advertise intent to publish

    def __init__(self,
                v0 = np.array([1,1]), #(rx,ry,r)
                v1 = np.array([-1,-1]),
                v2 = np.array([1,-1]),
                v3 = np.array([-1,1]),
                v4 = np.array([1,0]),
                v5 = np.array([-1,0]),
                v6 = np.array([1, 1]),
                v7 = np.array([1,1]),
                w = np.array([[0.0], [1.0], [1.0], [0.0], # sensor mask [100] - to deselect; [0] - to select.
                            [0.0], [1.0], [1.0], [0.0]]),
                lab = True,
                debug=False):

        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.v4 = v4
        self.v5 = v5
        self.v6 = v6
        self.v7 = v7
        self.debug = debug
        self.roll = 0.0
        self.pitch = 0.0
        self.forward = 0.0
        self.lab = lab
        self.minPoints = 3

        self.M = np.array([[44.80, 142.8, 47.54, 138],
        [45.40, 134.7, 45.19, 131],
        [45.40, 129.0, 45.19, 131],
        [50.97, 118.2, 47.54, 119],
        [49.40, 119.2, 47.57, 119],
        [50.23, 125.6, 45.19, 122],
        [40.57, 119.8, 45.19, 122],
        [47.93, 136.9, 47.54, 138]])
        self.M = self.M/100

        self.updated = np.array([False, False, False, False, False, False, False, False])
        # FLU
        self.orient = [-np.pi/2 + np.pi/2.25, -np.pi/2 + np.pi/12, -np.pi/2 - np.pi/12, -np.pi/2 - np.pi/2.25, np.pi/2 + np.pi/2.25, np.pi/2 + np.pi/12, np.pi/2 - np.pi/12, np.pi/2 - np.pi/2.25]
        self.remap = np.array([4, 5, 6, 7, 0, 1, 2, 3])
        # coordinate system?
        # XYZ - FLU
        self.offset = np.array([[0.08565, -0.03456, 0],
        [0.03333, -0.05809, 0],
        [-0.01433, -0.05809, 0],
        [-0.21079, -0.03456, 0],
        [-0.21079, 0.03456, 0],
        [-0.01433, 0.05809, 0],
        [0.03333, 0.05809, 0],
        [0.08565, 0.03456, 0]])

        self.w = w # sensor mask for lsql
        self.lamda = 0.045

        self.update_rate = 25.0

        # saves precious computation by calculating only when sufficient ranges are valid
        self.publishByValid = False
        # control publishing rate
        self.publishTimeNow = rospy.Time.now()
        self.publishTimeLast = rospy.Time.now()
        self.publishInterval = 1.0 / self.update_rate

        self.bodyXYZ = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        self.lpfilt = [low_pass.lowpassfilter(1.0/self.update_rate, 0.01) for i in range(8)]

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=1)
        self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=1)
        self.errorDz_pub = rospy.Publisher("error_dz", Float32, queue_size=1)
        self.errorDr_pub = rospy.Publisher("roll", Float32, queue_size=1)
        self.errorDp_pub = rospy.Publisher("pitch", Float32, queue_size=1)
        self.resultLSQ_pub = rospy.Publisher("resultLSQ", Float32MultiArray, queue_size=1)

        rospy.Subscriber("hub_1/ranges_raw", RangeArray, self.updatePolygonVertex, queue_size=1)
        # rospy.Subscriber("teraranger1/laser/scan", LaserScan, self.updatePolygonVertex, 0)
        # rospy.Subscriber("teraranger2/laser/scan", LaserScan, self.updatePolygonVertex, 1)
        # rospy.Subscriber("teraranger3/laser/scan", LaserScan, self.updatePolygonVertex, 2)
        # rospy.Subscriber("teraranger4/laser/scan", LaserScan, self.updatePolygonVertex, 3)
        # rospy.Subscriber("teraranger5/laser/scan", LaserScan, self.updatePolygonVertex, 4)
        # rospy.Subscriber("teraranger6/laser/scan", LaserScan, self.updatePolygonVertex, 5)

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.updateRPY)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self.bodyXYZ = self.bodyRotation(-self.pitch, -self.roll) #update the body rotation matrix
            self.publishRPY()
            # self.bodyXYZ = self.bodyRotation(-self.pitch, -self.roll) #update the body rotation matrix
            # self.publishRPY()
            # if (self.publish):
            #     self.lsqline_pub(np.copy(self.updated))
            rate.sleep()

    def sensorComp(self, old, i):
        M = self.M
        A = np.array([[M[i,0], 1],[M[i,1], 1]])
        Y = np.array([M[i,2], M[i,3]])
        X = np.dot(np.linalg.inv(A),Y)
        new = X[0]*old + X[1]
        return new

    def bodyRotation(self, pitch, roll, debug = False):
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
        self.forward = 0.0 #0.1 * local_position.pose.position.y
        q = local_position.pose.orientation
        euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
        self.roll = euler[0] #offset of 1 deg
        self.pitch = euler[1]

        # self.publishRPY()
        # self.errorDr_pub.publish(self.roll)
        # self.errorDp_pub.publish(self.pitch)

        if debug or self.debug:
            print 'roll ', self.roll, '\t pitch ', self.pitch, '\t yaw ', -(euler[2]-np.pi/2)

    def publishRPY(self, debug=False):
        self.errorDr_pub.publish(1.0*self.roll)
        self.errorDp_pub.publish(1.0*self.pitch)

    def updatePolygonVertex(self, msg, debug=False):
        self.updated = np.array([False, False, False, False, False, False, False, False])
        ranges = msg.ranges
        sensorCount = 8
        for i in range(sensorCount):
            j = self.remap[i]
            v = ranges[j].range
            if (debug):
                print 'teraranger' , i, 'distance ', v
            v = self.sensorComp(v,i)
            # v = self.medfilt[i].update_filter(v)
            self.updatePolygonVertex_old(v, i)

        updated = np.copy(self.updated)
        if (sum([updated[1], updated[2], updated[5], updated[6]]) >= self.minPoints):
            self.publishByValid = True
        # print sum([updated[1], updated[2], updated[5], updated[6]]), self.publishByValid
        self.publishTimeNow = rospy.Time.now()
        delta = self.publishTimeNow - self.publishTimeLast # from last published time
        # print delta.to_sec(), 0.8/self.update_rate
        publishByTime = delta.to_sec() > (0.8 * (1.0/self.update_rate)) #self.publishInterval
        if debug:
            print delta.to_sec(), (1.0/self.update_rate)
        if (self.publishByValid and publishByTime):
            self.lsqline_pub(updated, ranges)
            self.publishTimeLast = self.publishTimeNow

    def updatePolygonVertex_old(self, msg, index, debug=False):
        v = msg
        v_min = 20.0/1000.0
        v_max = 14.0
        update_check = v < v_min or v > v_max or np.isnan(v)
        if (not update_check):
            v = self.lpfilt[index].update_filter(v)
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
            elif index == 6:
                self.v6 = self.offset[index, 0:2] + self.rotate(v, self.orient[6])
                self.updated[6] = True
                if debug == True:
                    print '\n teraranger: ', index, '\t distance: ', v
                    print '\n teraranger: ', index, '\t distance: ', self.v6
            elif index == 7:
                self.v7 = self.offset[index, 0:2] + self.rotate(v, self.orient[7])
                self.updated[7] = True
                if debug == True:
                    print '\n teraranger: ', index, '\t distance: ', v
                    print '\n teraranger: ', index, '\t distance: ', self.v7
        else:
            self.updated[index] = False

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

    def lsqline_pub(self, updated, ranges, debug = False):
        # print 'sum there: ', sum(updated), updated
        rotm = euler_matrix(self.roll, self.pitch, 0.0, 'sxyz')
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

        B = np.array([self.v6[0], self.v6[1], 0])
        v6 = self.projectSubspace(A, B)
        v6 = np.dot(rotm[0:3,0:3], v6)
        if (debug):
            print 'v6', v6

        B = np.array([self.v7[0], self.v7[1], 0])
        v7 = self.projectSubspace(A, B)
        v7 = np.dot(rotm[0:3,0:3], v7)
        if (debug):
            print 'v7', v7

        A = np.array([[v0[0], -1, 0],
        [v1[0], -1, 0],
        [v2[0], -1, 0],
        [v3[0], -1, 0],
        [v4[0], 0, -1],
        [v5[0], 0, -1],
        [v6[0], 0, -1],
        [v7[0], 0, -1]])

        B = np.array([[-v0[1]],
        [-v1[1]],
        [-v2[1]],
        [-v3[1]],
        [-v4[1]],
        [-v5[1]],
        [-v6[1]],
        [-v7[1]]])

        # weights
        # step 1: initialise with the inf => 0 weight cus. exp(-inf)=0
        w = np.array([[np.inf],
        [np.inf],
        [np.inf],
        [np.inf],
        [np.inf],
        [np.inf],
        [np.inf],
        [np.inf]])

        ranges_tmp = np.array([[ranges[0].range],
        [ranges[1].range],
        [ranges[2].range],
        [ranges[3].range],
        [ranges[4].range],
        [ranges[5].range],
        [ranges[6].range],
        [ranges[7].range]])

        ranges_tmp = ranges_tmp[self.remap]

        w[updated] = ranges_tmp[updated]

        # step 2: scale it using weight - lamda
        w = self.lamda * w

        # print 'w_post: ', w

        # - exp of the squared weights
        w = np.exp(-np.square(w))

        # print 'w_post: ', w

        # mask the sensors with 1.0 or 0.0
        w = self.w * w

        # print w
        A = A * w
        B = B * w

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

        # print 'updated: ', updated
        # print 'ranges: ', (self.v0, self.v1, self.v2, self.v3, self.v4, self.v5, self.v6, self.v7)
        A = A[updated, ...] #mask outdated values
        B = B[updated, ...] #mask outdated values
        # print 'A: ', A
        # print 'B: ', B

        # print 'front valids: ', sum([updated[7], updated[0]])
        if (self.lab):
            if (sum([updated[7], updated[0]]) >= 2):
                A_ = np.array([[v0[0], -1],
                [v7[0], -1]])
                B_ = np.array([[-v0[1]],
                [-v7[1]]])
                x_ = np.linalg.lstsq(A_,B_)[0]

                alpha_ = np.arctan(x_[0])
                self.forward = x_[1] * np.cos(alpha_)
                # print 'forward: ', self.forward, ' alpha: ', alpha_
                self.errorDx_pub.publish(-abs(self.forward))
                # self.updated[0] = False
                # self.updated[7] = False

        # At = A.transpose()
        #
        # x = np.dot(np.linalg.inv(np.dot(At, A)), np.dot(At, B))
        # print updated
        if (sum([updated[1], updated[2], updated[5], updated[6]]) >= self.minPoints):

            x = np.linalg.lstsq(A,B)[0]

            alpha = np.arctan(x[0])
            rR = x[1] * np.cos(alpha)
            rL = x[2] * np.cos(alpha)

            res = Float32MultiArray()
            res.data = [rL, rR, alpha]

            width = abs(rL) + abs(rR)
            dy = (width/2) - rL

            dx = self.forward

            if self.debug or debug:
                print 'rL: \t', rL
                print 'rR: \t', rR
                print 'yaw: \t', alpha
                print 'centre: \t', dy
                #print 'A: \t', A
                #print 'B: \t', B
                print 'x: \t', x

            if (not self.lab):
                self.errorDx_pub.publish(dx) #publish this seperately if doing lab experiment

            self.errorDy_pub.publish(dy)
            self.errorDz_pub.publish(alpha)
            self.resultLSQ_pub.publish(res)
            # self.updated[0] = False
            # self.updated[1] = False
            # self.updated[2] = False
            # # self.updated[3] = False
            # # self.updated[4] = False
            # self.updated[5] = False
            # self.updated[6] = False
            # self.updated[7] = False


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
