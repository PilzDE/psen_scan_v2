#!/usr/bin/env python

import sys
import unittest
from math import degrees

import rospy
from sensor_msgs.msg import LaserScan


DECIMAL_PLACE_ACCURACY = 6


class HwtestScanRange(unittest.TestCase):
    """ Check if the laserscan message covers the scan range as expected.
    """
    def setUp(self):
        rospy.init_node('hwtest_scan_range')
        self.received_msgs = []
        self.angle_start = rospy.get_param("~angle_start")
        self.angle_end = rospy.get_param("~angle_end")
        rospy.Subscriber("/laser_scanner/scan", LaserScan, self.callback)

    def callback(self, msg):
        self.received_msgs.append(msg)

    def test_one_message_received(self):
        while len(self.received_msgs) <= 0:
            rospy.sleep(.1)
        self.assertGreaterEqual(len(self.received_msgs), 1)

        message: LaserScan = self.received_msgs[0]

        self.assertAlmostEqual(self.angle_start, message.angle_min, DECIMAL_PLACE_ACCURACY,
                               "angle_min of the laserscan message is " + str(message.angle_min) +
                               " but should be " + str(self.angle_start) + ".")
        angle_max = message.angle_min + message.angle_increment * (len(message.ranges)-1)
        self.assertAlmostEqual(self.angle_end, angle_max, DECIMAL_PLACE_ACCURACY,
                               "angle_max of the laserscan message is " + str(angle_max) +
                               " but should be " + str(self.angle_end) + ".")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('psen_scan_v2', 'hwtest_scan_range', HwtestScanRange)
