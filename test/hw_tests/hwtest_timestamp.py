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

import rospy
from sensor_msgs.msg import LaserScan


DESIRED_NUMBER_OF_STAMPS = 50
WAIT_TIMEOUT_S = 5


def is_less_than(l1: list, l2: list):
    """ Check if every element of list l1 is smaller than the element of list l2 at the same position.
    """
    return all([a < b for (a, b) in zip(l1, l2)])


def is_increasing(values: list):
    """ Check if the given list consists of monotonically increasing values.
    """
    return is_less_than(values[:-1], values[1:])


class HwtestTimestamp(unittest.TestCase):
    """ Check if the timestamp of the laserscan messages
        - monotonically increases,
        - is lower than the time the msg was received.
    """
    def setUp(self):
        rospy.init_node('hwtest_timestamp')
        self.received_stamps = []
        self.receipt_times = []
        rospy.Subscriber("/laser_1/scan", LaserScan, self.callback)

    def callback(self, msg):
        if len(self.received_stamps) < DESIRED_NUMBER_OF_STAMPS:
            self.receipt_times.append(rospy.get_rostime())
            self.received_stamps.append(msg.header.stamp)

    def wait_for_msgs(self):
        start_wait = rospy.Time.now()
        while len(self.received_stamps) < DESIRED_NUMBER_OF_STAMPS and not rospy.is_shutdown():
            self.assertLess((rospy.Time.now() - start_wait).to_sec(), WAIT_TIMEOUT_S,
                            "Could not gather enough timestamps.")
            rospy.sleep(.1)

    def test_timestamp_increasing(self):
        self.wait_for_msgs()
        self.assertTrue(is_increasing(self.received_stamps), "Timestamp is not monotonically increasing")

    def test_timestamp_upper_bound(self):
        self.wait_for_msgs()
        self.assertEqual(len(self.received_stamps), len(self.receipt_times), "Lists have incompatible length")
        self.assertTrue(is_less_than(self.received_stamps, self.receipt_times),
                        "Timestamp is not lower than the time the msg was received.")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('psen_scan_v2', 'hwtest_timestamp', HwtestTimestamp)
