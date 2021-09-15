#!/usr/bin/env python3

import rospy

def server():
    x=0xD80E
    x=((x << 8) | (x >> 8)) & 0xFFFF
    print(x)


if __name__ == '__main__':
    try:
        server()
    except rospy.ROSInterruptException:
        pass


