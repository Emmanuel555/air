#!/usr/bin/python

import rospy
import numpy as np

from std_msgs.msg import Float32
from sensor_msgs.msg import Range, LaserScan
from rospy.numpy_msg import numpy_msg

# simple class to contain the node's variables and code
class CentroidFinder:     # class constructor; subscribe to topics and advertise intent to publish

    def __init__(self, v0 = np.array([1,1]), v1 = np.array([1,1]), v2 = np.array([1,1]), v3 = np.array([1,1]), debug=False ):

        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.debug = debug

        self.v_list = np.array([self.v0, self.v1, self.v2, self.v3])

        self.updated = [False, False, False, False]
        self.orient = [np.pi/4, -3*np.pi/4, -np.pi/4, 3*np.pi/4]
        self.update_rate = 100

        rospy.Subscriber("teraranger1/laser/scan", LaserScan, self.updatePolygonVertex, 0)
        rospy.Subscriber("teraranger2/laser/scan", LaserScan, self.updatePolygonVertex, 1)
        rospy.Subscriber("teraranger3/laser/scan", LaserScan, self.updatePolygonVertex, 2)
        rospy.Subscriber("teraranger4/laser/scan", LaserScan, self.updatePolygonVertex, 3)

        self.errorDx_pub = rospy.Publisher("error_dx", Float32, queue_size=10)
        self.errorDy_pub = rospy.Publisher("error_dy", Float32, queue_size=10)
        self.errorDz_pub = rospy.Publisher("error_dz", Float32, queue_size=10)

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            # if (self.updated[0]==True and self.updated[1]==True and self.updated[2]==True and self.updated[3]==True):
            self.v_list = np.array([self.v0, self.v3, self.v1, self.v2, self.v0])
            self.centroid_pub(self.v_list);
            rate.sleep()

    def updatePolygonVertex(self, msg, index):
        v = msg.ranges[0]
        if index == 0:
            self.v0 = self.rotate(v, self.orient[0])
            self.updated[0] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v0
        elif index == 1:
            self.v1 = self.rotate(v, self.orient[1])
            self.updated[1] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v1
        elif index == 2:
            self.v2 = self.rotate(v, self.orient[2])
            self.updated[2] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v2
        elif index == 3:
            self.v3 = self.rotate(v, self.orient[3])
            self.updated[3] = True
            if self.debug == True:
                print '\n teraranger: ', index, '\t distance: ', v
                print '\n teraranger: ', index, '\t distance: ', self.v3

    def rotate(self, r, angle):
        x = r * np.cos(angle)
        y = r * np.sin(angle)

        return np.array([x,y])

    def area(self, v):
        tmp_area = 0
        for i in range(len(v)-1):
            xi = v[i,0]
            yi = v[i,1]
            xi1 = v[i+1,0]
            yi1 = v[i+1,1]
            f = (xi*yi1 - xi1*yi)
            tmp_area += f
        tmp_area /= 2
        return tmp_area

    def centroid(self, v, area):
        x = 0
        y = 0
        for i in range(len(v)-1):
            xi = v[i,0]
            yi = v[i,1]
            xi1 = v[i+1,0]
            yi1 = v[i+1,1]
            f = (xi*yi1 - xi1*yi)
            x += ((xi + xi1) * f)
            y += ((yi + yi1) * f)
        x /= (6*area)
        y /= (6*area)
        return np.array([x,y])

    def centre_of_mass(self,v):
        v = np.asarray(v)
        n = len(v)
        #remove the repeated last vertex first
        v = np.delete(v, n-1, 0)

        #sanity check -- to solve inf value problem.
        #the sum (v) should be < n * max-range
        inf_chk = np.isinf(v)
        if np.any(inf_chk):
            return np.array([0,0])
        else:
            centre = sum(v)/ len(v)
            return centre

    def yaw(self):
        dy = self.v0[1] - self.v3[1]

        dx = self.v0[0] - self.v3[0]
        yaw = np.arctan(dy/dx)
        # print yaw
        # yaw = np.arctan(float(dy)/(float(dx)))
        return yaw

    def centroid_pub(self, v):
        area = self.area(v)
        # centre = self.centroid(v, area)
        centre = self.centre_of_mass(v)
        yaw = self.yaw()

        self.updated[0] = False
        self.updated[1] = False
        self.updated[2] = False
        self.updated[3] = False



        if self.debug == True:
            print 'area: \t\t', area
            print 'centre: \t', centre

        self.errorDx_pub.publish(centre[0])
        self.errorDy_pub.publish(centre[1])
        self.errorDz_pub.publish(yaw)

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
