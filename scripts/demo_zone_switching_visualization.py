#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Joy


speed_scale = 1.0

def joy_callback(data):
    global speed_scale
    speed_scale = max([abs(data.axes[1]), abs(data.axes[2]), 0.3])

def get_marker(ns,color,scale=1.0, z_offset=0):
    global speed_scale
    marker = Marker()
    marker.header.frame_id = "scan"
    marker.ns = ns
    marker.type = marker.TRIANGLE_LIST
    marker.action = marker.ADD
    marker.scale.x = scale * speed_scale
    marker.scale.y = scale * speed_scale
    marker.scale.z = scale * speed_scale
    marker.color.a = 1.0

    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = z_offset
    
    poly = [[0,-1],[2,-1],[2.2,-0.8], [2.2,0.8], [2,1], [0,1]]
    for p1,p2 in zip(poly, poly[1:]):
        marker.points.append(Point(0,0,0))
        marker.points.append(Point(p1[0],p1[1],0))
        marker.points.append(Point(p2[0],p2[1],0))
        marker.colors.append(color)
    return marker

def talker():
    pub = rospy.Publisher('zone', Marker, queue_size=10)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.init_node('zone_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    
        #poly.polygon.points.append(Point32(1,1,0))
        pub.publish(get_marker("error",color=ColorRGBA(1,0,0,1),scale=0.3))
        pub.publish(get_marker("warn",ColorRGBA(1,1,0,1),scale=0.4,z_offset=0.01))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass