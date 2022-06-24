#!/usr/bin/env python3
import random

import rospy
from std_msgs.msg import Byte

if __name__ == "__main__":
    rospy.init_node("multiple_zones_test")

    pub = rospy.Publisher("/relay_cmd", Byte)

    r1 = rospy.Rate(1)  # 1hz
    r10 = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(0)
        r10.sleep()
        pub.publish(8)
        r10.sleep()
        pub.publish(0)
        if random.random() > .5:
            r10.sleep()
            pub.publish(8)
        r1.sleep()
