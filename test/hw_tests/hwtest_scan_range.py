#!/usr/bin/env python
# Copyright (c) 2021 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import sys
import unittest
from math import floor

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
        self.resolution = rospy.get_param("~resolution")
        rospy.Subscriber("/laser_1/scan", LaserScan, self.callback)

    def callback(self, msg):
        self.received_msgs.append(msg)

    def test_one_message_received(self):
        while len(self.received_msgs) <= 5:  # Avoid testing a message with an old config (can happen at scanner start)
            rospy.sleep(.1)

        self.assertTrue(self.received_msgs)
        message: LaserScan = self.received_msgs[-1]

        num_intervals = floor((self.angle_end - self.angle_start) / self.resolution)
        expected_max_angle = self.angle_start + self.resolution * num_intervals

        self.assertAlmostEqual(self.angle_start, message.angle_min, DECIMAL_PLACE_ACCURACY,
                               "angle_min of the laserscan message is " + str(message.angle_min) +
                               " but should be " + str(self.angle_start) + ".")
        self.assertAlmostEqual(expected_max_angle, message.angle_max, DECIMAL_PLACE_ACCURACY,
                               "angle_max of the laserscan message is " + str(message.angle_max) +
                               " but should be " + str(expected_max_angle) + ".")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('psen_scan_v2', 'hwtest_scan_range', HwtestScanRange)
