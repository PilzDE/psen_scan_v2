#!/usr/bin/env python3
import random

import rospy
from std_msgs.msg import Byte

if __name__ == "__main__":
    rospy.init_node("multiple_zones_test")

    pub = rospy.Publisher("/relay_cmd", Byte, queue_size=10)

    def publish_zone_and_sleep(zone: int):
        if zone == 0:
            pub.publish(0)
        elif zone == 1:
            pub.publish(8)
        else:
            raise NotImplementedError("Only zones 0 and 1 are supported.")
        r10.sleep()

    r1 = rospy.Rate(1)  # 1hz
    r10 = rospy.Rate(8)  # 10hz
    while not rospy.is_shutdown():
        publish_zone_and_sleep(0)
        publish_zone_and_sleep(1)
        publish_zone_and_sleep(0)
        if random.random() > .5:
            publish_zone_and_sleep(1)
        r1.sleep()
