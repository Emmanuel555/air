#!/usr/bin/python

# import main ROS python library

import rospy
import numpy as np
import med_filter
import low_pass

from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix, euler_from_matrix
from teraranger_array.msg import RangeArray
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float32MultiArray, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, PoseStamped
# import the Float32 message type

from sensor_msgs.msg import LaserScan

# simple class to contain the node's variables and code

class TROneNode:     # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        self.v0 = np.array([1,1])
        self.v1 = np.array([1,1])
        self.v2 = np.array([1,1])
        self.v3 = np.array([1,1])
        self.v4 = np.array([1,1])
        self.v5 = np.array([1,1])
        self.v6 = np.array([1,1])
        self.v7 = np.array([1,1])
        self.debug = False
        self.roll = 0.0
        self.pitch = 0.0
        self.forward = 0.0
        self.updated = np.array([False, False, False, False, False, False, False, False])

        self.update_rate = 20 # hertz
        self.sensorCount = 8
#        self.update_timer = 1/ (self.update_rate)
        self.sensor = []

        self.xyt = [0.0, 0.0, 0.0]

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

        self.M = np.array([[44.80, 142.8, 47.54, 138],
        [45.40, 134.7, 45.19, 131],
        [45.40, 129.0, 45.19, 131],
        [50.97, 118.2, 47.54, 119],
        [49.40, 119.2, 47.57, 119],
        [50.23, 125.6, 45.19, 122],
        [40.57, 119.8, 45.19, 122],
        [47.93, 136.9, 47.54, 138]])
        self.M = self.M/100

        self.orient = [-np.pi/2 + np.pi/2.25, -np.pi/2 + np.pi/12, -np.pi/2 - np.pi/12, -np.pi/2 - np.pi/2.25, np.pi/2 + np.pi/2.25, np.pi/2 + np.pi/12, np.pi/2 - np.pi/12, np.pi/2 - np.pi/2.25]
        self.remap = [6, 5, 4, 3, 2, 1, 0, 7]
        print 'orietation initialized as' , self.orient

        self.medfilt = [med_filter.medfilter(20) for i in range(self.sensorCount)]
        self.lpfilt = [low_pass.lowpassfilter(1/15.0, 0.75) for i in range(self.sensorCount)]

        try:
            #print i    # advertise that we'll publish on the sum and moving_average topics
            self.range_pub = [rospy.Publisher("tr%d" %(i+1), Range, queue_size=1) for i in range(self.sensorCount)]
            self.range_mf_pub = [rospy.Publisher("tr_mf%d" %(i+1), Range, queue_size=1) for i in range(self.sensorCount)]
            self.range_lf_pub = [rospy.Publisher("tr_lp%d" %(i+1), Range, queue_size=1) for i in range(self.sensorCount)]
            # world frame
            self.range_w_pub = [rospy.Publisher("W_tr%d" %(i+1), Range, queue_size=1) for i in range(self.sensorCount)]
        except:
            print "error initializing terarangers"

        rospy.Subscriber("hub_1/ranges_raw", RangeArray, self.updatePolygonVertex, queue_size=1)
        # rospy.Subscriber("error_dx", Float32, self.updateXYT, 0, queue_size=1)
        # rospy.Subscriber("error_dy", Float32, self.updateXYT, 1, queue_size=1)
        # rospy.Subscriber("error_dz", Float32, self.updateXYT, 2, queue_size=1)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.updatePose, queue_size=1)
        rospy.Subscriber("resultLSQ", Float32MultiArray, self.drawTunnel, queue_size=1)

        self.tunnel_pub = rospy.Publisher("tunnel_marker", Marker)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            #self.timer_callback()
            self.bodyXYZ = self.bodyRotation(-self.pitch, -self.roll)
            self.lsqline_pub()
            rate.sleep()

        # create the Timer with period self.moving_average_period
#        rospy.Timer(rospy.Duration(self.update_timer, self.timer_callback))

        # print out a message for debugging
#        rospy.loginfo("Created terarangers publishing node with period of %f seconds", self.update_timer)

    # the callback function for the timer event
    def timer_callback(self):         # create the message containing the moving average

        for i in range(len(self.sensor)):
            #print "publishing"
            distance = self.sensor[i].readRangeData()
            #print distance
            #if (distance < 14000 and distance > 200):
            terarangers_msg = LaserScan()
            terarangers_msg.header.frame_id = "base_range"
            terarangers_msg.header.stamp = rospy.Time.now()
            terarangers_msg.angle_min = 0
            terarangers_msg.angle_max = 0
            terarangers_msg.angle_increment = 0
            terarangers_msg.time_increment = 0 # 14 metres
            terarangers_msg.scan_time = 0
            terarangers_msg.range_min = 0.2
            terarangers_msg.range_max = 14.0
            terarangers_msg.ranges = [distance/1000.0]
            terarangers_msg.intensities = [0]
                # publish the moving average
            self.range_pub[i].publish(terarangers_msg)
            rospy.sleep(1.)

    def updateXYT(self, msg, args, debug=False):
        self.xyt[args] = msg.data
        br = TransformBroadcaster()
        br.sendTransform((self.xyt[0], self.xyt[1], 0.0),
                        quaternion_from_euler(0, 0, self.xyt[2]),
                        rospy.Time.now(),
                        "base_link",
                        "map")
        if (debug):
            print 'x: ' , self.xyt[0], 'y: ', self.xyt[1], 't: ', self.xyt[2]

    def updatePose(self, msg, debug=False):
        pose = msg.pose.position
        quat = msg.pose.orientation

        euler = np.array(euler_from_quaternion((quat.x, quat.y, quat.z, quat.w)))
        self.roll = euler[0] #offset of 1 deg
        self.pitch = euler[1]

        br = TransformBroadcaster()
        # trhub_link -> base_link: only quaternion
        euler_ = euler_from_matrix(self.bodyXYZ)
        br.sendTransform((0, 0, 0.0),
                        (quaternion_from_euler(euler_[0],euler_[1],0)),
                        rospy.Time.now(),
                        "w_trhub_link",
                        "trhub_link")

        br.sendTransform((0, 0, 0.1),
                        (quat.x, quat.y, quat.z, quat.w),
                        rospy.Time.now(),
                        "trhub_link",
                        "base_link")
        #base_link -> map: only translation
        br = TransformBroadcaster()
        br.sendTransform((pose.x, pose.y, pose.z),
                        (0, 0, 0, 1),
                        rospy.Time.now(),
                        "base_link",
                        "map")
        if (debug):
            print 'x: ' , self.xyt[0], 'y: ', self.xyt[1], 't: ', self.xyt[2]

    def drawTunnel(self, msg, debug=False):
        width = abs(msg.data[0]) + abs(msg.data[1])
        box = Marker()
        box.header = Header()
        box.header.frame_id = "map"
        box.header.stamp = rospy.Time.now()
        box.id = 0
        box.type = 1 #CUBE=1
        box.action = 0
        q = quaternion_from_euler(0,0,np.pi/2)
        box.pose = Pose(Point(0,0,0), Quaternion(q[0], q[1], q[2], q[3]))
        box.scale = Vector3(10,width,20)
        box.color = ColorRGBA(0.0, 1.0, 0.0, 0.2)
        self.tunnel_pub.publish(box)


    def sensorComp(self, old, i):
        M = self.M
        A = np.array([[M[i,0], 1],[M[i,1], 1]])
        Y = np.array([M[i,2], M[i,3]])
        X = np.dot(np.linalg.inv(A),Y)
        new = X[0]*old + X[1]
        return new

    def updatePolygonVertex(self, msg, debug=False):
        ranges = msg.ranges
        sensorCount = 8

        terarangers_msg = Range()
        terarangers_msg.radiation_type = 1
        terarangers_msg.min_range = 0.2
        terarangers_msg.max_range = 14.0
        terarangers_msg.field_of_view = 0.0524 / 5.0

        br = TransformBroadcaster()

        for i in range(sensorCount):
            #remap the sensor_msgs
            j = self.remap[i]

            v = ranges[j].range
            v = self.sensorComp(v,i)


            terarangers_msg.header.frame_id = "sensor%d_link" %i
            terarangers_msg.header.stamp = rospy.Time.now()
            terarangers_msg.range = v

            if (debug):
                #print 'teraranger' , i, 'distance ', v
                print 'teraranger' , i, 'remapped ', j
            #v = self.sensorComp(v,i)

            self.range_pub[i].publish(terarangers_msg)
            tmp = v
            v_min = 20.0/1000.0
            v_max = 14.0
            if (v > v_min or v < v_max or False):
                v = self.medfilt[i].update_filter(tmp)
                terarangers_msg.range = v
                self.range_mf_pub[i].publish(terarangers_msg)
                v = self.lpfilt[i].update_filter(tmp)
                terarangers_msg.range = v
                self.range_lf_pub[i].publish(terarangers_msg)

            br.sendTransform((self.offset[i][0], self.offset[i][1], self.offset[i][2]),
                            quaternion_from_euler(0, 0, self.orient[i]),
                            rospy.Time.now(),
                            "sensor%d_link" %i,
                            "trhub_link")
            self.updatePolygonVertex_old(v, i)

    def updateWorld(self, msg, debug=False):
        ranges = msg
        sensorCount = 8

        terarangers_msg = Range()
        terarangers_msg.radiation_type = 1
        terarangers_msg.min_range = 0.2
        terarangers_msg.max_range = 14.0
        terarangers_msg.field_of_view = 0.0524 / 5.0

        br = TransformBroadcaster()

        for i in range(sensorCount):

            terarangers_msg.header.frame_id = "w_sensor%d_link" %i
            terarangers_msg.header.stamp = rospy.Time.now()
            terarangers_msg.range = np.hypot(ranges[i][0], ranges[i][1])

            self.range_w_pub[i].publish(terarangers_msg)
            br.sendTransform((0.0, 0.0, self.offset[i][2]),
                            quaternion_from_euler(0, 0, np.arctan2(ranges[i][1], ranges[i][0])),
                            rospy.Time.now(),
                            "w_sensor%d_link" %i,
                            "w_trhub_link")
            # self.updatePolygonVertex_old(v, i)

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

    def updatePolygonVertex_old(self, msg, index, debug=False):
        v = msg
        v_min = 20.0/1000.0
        v_max = 14.0
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

        self.updateWorld([v0,v1,v2,v3,v4,v5,v6,v7])

        # A = np.array([[v0[0], -1, 0],
        # [v1[0], -1, 0],
        # [v2[0], -1, 0],
        # [v3[0], -1, 0],
        # [v4[0], 0, -1],
        # [v5[0], 0, -1],
        # [v6[0], 0, -1],
        # [v7[0], 0, -1]])
        #
        # B = np.array([[-v0[1]],
        # [-v1[1]],
        # [-v2[1]],
        # [-v3[1]],
        # [-v4[1]],
        # [-v5[1]],
        # [-v6[1]],
        # [-v7[1]]])
        #
        # #weights
        # # w = np.array([[v0[0]],
        # # [v1[0]],
        # # [v2[0]],
        # # [v3[0]],
        # # [v4[0]],
        # # [v5[0]],
        # # [v6[0]],
        # # [v7[0]]])
        # w = np.array([[100],
        # [0.26],
        # [0.26],
        # [100],
        # [100],
        # [0.26],
        # [0.26],
        # [100]])
        # w = np.exp(-np.square(w))
        # # w = np.sqrt(abs(w))
        #
        # # print w
        # A = A * w
        # B = B * w
        #
        # # A = np.array([[self.v0[0], -1, 0],
        # # [self.v1[0], -1, 0],
        # # [self.v2[0], -1, 0],
        # # [self.v3[0], 0, -1],
        # # [self.v4[0], 0, -1],
        # # [self.v5[0], 0, -1]])
        # #
        # # B = np.array([[-self.v0[1]],
        # # [-self.v1[1]],
        # # [-self.v2[1]],
        # # [-self.v3[1]],
        # # [-self.v4[1]],
        # # [-self.v5[1]]])
        #
        # updated = np.copy(self.updated)
        # # print updated
        # # print 'ranges: ', (self.v0, self.v1, self.v2, self.v3, self.v4, self.v5, self.v6, self.v7)
        # A = A[updated, ...] #mask outdated values
        # B = B[updated, ...] #mask outdated values
        # # print 'A: ', A
        # # print 'B: ', B
        #
        # self.updated[0] = False
        # self.updated[1] = False
        # self.updated[2] = False
        # self.updated[3] = False
        # self.updated[4] = False
        # self.updated[5] = False
        # self.updated[6] = False
        # self.updated[7] = False
        #
        # # print 'front valids: ', sum([updated[7], updated[0]])
        #
        # if (sum([updated[7], updated[0]]) >= 2):
        #     A_ = np.array([[v0[0], -1],
        #     [v7[0], -1]])
        #     B_ = np.array([[-v0[1]],
        #     [-v7[1]]])
        #     x_ = np.linalg.lstsq(A_,B_)[0]
        #
        #     alpha_ = np.arctan(x_[0])
        #     self.forward = x_[1] * np.cos(alpha_)
        #     # print 'forward: ', self.forward, ' alpha: ', alpha_
        #     self.errorDx_pub.publish(-abs(self.forward))
        #
        # # At = A.transpose()
        # #
        # # x = np.dot(np.linalg.inv(np.dot(At, A)), np.dot(At, B))
        # # print updated
        # if (sum([updated[1], updated[2], updated[5], updated[6]]) >= 4):
        #
        #     x = np.linalg.lstsq(A,B)[0]
        #
        #     alpha = np.arctan(x[0])
        #     rR = x[1] * np.cos(alpha)
        #     rL = x[2] * np.cos(alpha)
        #
        #     res = Float32MultiArray()
        #     res.data = [rL, rR, alpha]
        #
        #     width = abs(rL) + abs(rR)
        #     dy = (width/2) - rL
        #
        #     dx = self.forward
        #
        #     if self.debug or debug:
        #         print 'rL: \t', rL
        #         print 'rR: \t', rR
        #         print 'yaw: \t', alpha
        #         print 'centre: \t', dy
        #         #print 'A: \t', A
        #         #print 'B: \t', B
        #         print 'x: \t', x
        #
        #     # self.errorDx_pub.publish(dx) #publish this seperately
        #     self.errorDy_pub.publish(dy)
        #     self.errorDz_pub.publish(alpha)
        #     self.resultLSQ_pub.publish(res)

if __name__ == "__main__":     # initialize the ROS client API, giving the default node name

    rospy.init_node("hub_vis_node")

    node = TROneNode()

    # enter the ROS main loop
    # rospy.spin()
