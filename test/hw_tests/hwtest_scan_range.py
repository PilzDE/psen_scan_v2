#!/usr/bin/env python

import sys
import unittest
from math import degrees

import rospy
from sensor_msgs.msg import LaserScan

import pytest

PLACES = 6


class HwtestScanRange(unittest.TestCase):
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
        rospy.loginfo(f"param angle_start:\n {degrees(self.angle_start)}")
        rospy.loginfo(f"param angle_end:\n {degrees(self.angle_end)}")
        rospy.loginfo(f"msg angle_min:\n {degrees(message.angle_min)}")
        rospy.loginfo(f"msg angle_max:\n {degrees(message.angle_max)}")
        rospy.loginfo(
            f"msg angle_increment:\n {degrees(message.angle_increment)}")
        rospy.loginfo("msg angle_increment * len(ranges):\n" +
                      f" {degrees(message.angle_increment * len(message.ranges))}")
        rospy.loginfo("msg angle_increment * (len(ranges)-1):\n" +
                      f" {degrees(message.angle_increment * (len(message.ranges)-1))}")
        rospy.loginfo("msg angle_min + angle_increment * (len(ranges)-1):\n" +
                      f" {degrees(message.angle_min + message.angle_increment * (len(message.ranges)-1))}")

        self.assertAlmostEqual(self.angle_start, message.angle_min, PLACES)
        self.assertAlmostEqual(self.angle_end,
                               message.angle_min + message.angle_increment *
                               (len(message.ranges)-1),
                               PLACES)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('psen_scan_v2', 'hwtest_scan_range', HwtestScanRange)
