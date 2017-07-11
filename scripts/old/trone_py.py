#!/usr/bin/python

# import main ROS python library

import rospy
import mraa
import teraranger

# import the Float32 message type

from sensor_msgs.msg import Range

# simple class to contain the node's variables and code

class TROneNode:     # class constructor; subscribe to topics and advertise intent to publish
    def __init__(self, rosstart = True ):
        self.sensorCount = 3 # the number of sensors to attempt to add
        self.update_rate = 10 # hertz
#        self.update_timer = 1/ (self.update_rate)
        self.sensor = []

        try:
            self.sensor = [teraranger.TeraRangerOne(address=(0x31 + i)) for i in range(self.sensorCount)]
                # advertise that we'll publish on the sum and moving_average topics
            self.range_pub = [rospy.Publisher("teraranger%d" %(i), Range) for i in range(self.sensorCount)]
        except:
            print "error initializing terarangers"

        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self.timer_callback()
            rate.sleep()

        # create the Timer with period self.moving_average_period
#        rospy.Timer(rospy.Duration(self.update_timer, self.timer_callback))

        # print out a message for debugging
#        rospy.loginfo("Created terarangers publishing node with period of %f seconds", self.update_timer)

    # the callback function for the timer event
    def timer_callback(self):         # create the message containing the moving average

        for i in range(len(self.sensor)):
            # print "publishing"
            distance = self.sensor[i].readRangeData()
            if (distance < 14000 and distance > 200):
                terarangers_msg = Range()
                terarangers_msg.header.frame_id = "base_range"
                terarangers_msg.header.stamp = rospy.Time.now()
                terarangers_msg.radiation_type = 1
                terarangers_msg.field_of_view = 0.0593
                terarangers_msg.min_range = 200
                terarangers_msg.max_range = 14000 # 14 metres
                terarangers_msg.range = distance
                # publish the moving average
                self.range_pub[i].publish(terarangers_msg)

if __name__ == "__main__":     # initialize the ROS client API, giving the default node name

    rospy.init_node("teraranger_node")

    node = TROneNode()

    # enter the ROS main loop
#    rospy.spin()
