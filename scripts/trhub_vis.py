#!/usr/bin/python

# import main ROS python library

import rospy
import numpy as np

from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
from teraranger_array.msg import RangeArray
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

# import the Float32 message type

from sensor_msgs.msg import LaserScan

# simple class to contain the node's variables and code

class TROneNode:     # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self):
        self.update_rate = 1 # hertz
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

        try:
            #print i    # advertise that we'll publish on the sum and moving_average topics
            self.range_pub = [rospy.Publisher("tr%d" %(i+1), Range, queue_size=1) for i in range(self.sensorCount)]
        except:
            print "error initializing terarangers"

        rospy.Subscriber("hub_1/ranges_raw", RangeArray, self.updatePolygonVertex, queue_size=1)
        rospy.Subscriber("error_dx", Float32, self.updateXYT, 0, queue_size=1)
        rospy.Subscriber("error_dy", Float32, self.updateXYT, 1, queue_size=1)
        rospy.Subscriber("error_dz", Float32, self.updateXYT, 2, queue_size=1)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            #self.timer_callback()
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
            br.sendTransform((self.offset[i][0], self.offset[i][1], self.offset[i][2]),
                            quaternion_from_euler(0, 0, self.orient[i]),
                            rospy.Time.now(),
                            "sensor%d_link" %i,
                            "base_link")
            #self.updatePolygonVertex_old(v, i)

if __name__ == "__main__":     # initialize the ROS client API, giving the default node name

    rospy.init_node("hub_vis_node")

    node = TROneNode()

    # enter the ROS main loop
    # rospy.spin()
