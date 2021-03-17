#!/usr/bin/env python

import unittest
import rospy

class TestStartup(unittest.TestCase):
    def test_startup(self):
        i = 0
        expectedTopic = ['/laser_scanner/scan', 'sensor_msgs/LaserScan']
        while not expectedTopic in rospy.get_published_topics():
            rospy.sleep(rospy.Duration(0.5))
            self.assertLess(i, 10, msg="node did not come up in the expected time") # wait 5s max

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_psen_scan_v2_launch')
    rostest.rosrun('psen_scan_v2', 'test_startup', TestStartup)
