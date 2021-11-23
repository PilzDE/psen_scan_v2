#!/usr/bin/env python3
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

import unittest
from math import radians

import rospy
from sensor_msgs.msg import LaserScan


def to_multiple_of_tenth_degree_in_radian(radian_angle: float) -> float:
    reminder = radian_angle % radians(0.1)
    if (reminder < 0.5 * radians(0.1)):
        return radian_angle - reminder
    return radian_angle + reminder


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
        # Avoid testing a message with an old config (can happen at scanner start)
        while len(self.received_msgs) <= 5:
            rospy.sleep(.1)

        self.assertTrue(self.received_msgs)
        message: LaserScan = self.received_msgs[-1]

        angle_start_rounded = to_multiple_of_tenth_degree_in_radian(self.angle_start)
        angle_end_rounded = to_multiple_of_tenth_degree_in_radian(self.angle_end)
        resolution_rounded = to_multiple_of_tenth_degree_in_radian(self.resolution)

        expected_max_angle = angle_end_rounded - \
            ((angle_end_rounded - angle_start_rounded) % resolution_rounded)

        self.assertAlmostEqual(angle_start_rounded, message.angle_min, DECIMAL_PLACE_ACCURACY,
                               "angle_min of the laserscan message is " + str(message.angle_min) +
                               " but should be " + str(angle_start_rounded) + ".")
        self.assertAlmostEqual(expected_max_angle, message.angle_max, DECIMAL_PLACE_ACCURACY,
                               "angle_max of the laserscan message is " + str(message.angle_max) +
                               " but should be " + str(expected_max_angle) + ".")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('psen_scan_v2', 'hwtest_scan_range', HwtestScanRange)
