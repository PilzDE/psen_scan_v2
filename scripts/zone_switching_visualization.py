#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Joy
from psen_scan_v2.msg import ZoneSetConfiguration


speed_scale = 1.0
active_zoneset_config = None

def zoneset_configuration_callback(zoneset_config):
    global active_zoneset_config
    active_zoneset_config = zoneset_config
    rospy.loginfo(rospy.get_caller_id())

def joy_callback(data):
    global speed_scale
    speed_scale = max([abs(data.axes[1]), abs(data.axes[2]), 0.3])

def get_marker(ns, points, id, color, scale=1.0, z_offset=0):
    global speed_scale
    marker = Marker()
    marker.header.frame_id = "scan"
    marker.ns = ns
    marker.id = id
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
    
    #poly = [[0,-1],[2,-1],[2.2,-0.8], [2.2,0.8], [2,1], [0,1]]
    for p1,p2 in zip(points, points[1:]):
        marker.points.append(Point(0,0,0))
        marker.points.append(Point(p1.x,p1.y,0))
        marker.points.append(Point(p2.x,p2.y,0))
        marker.colors.append(color)
    return marker

def talker():
    global active_zoneset_config
    pub = rospy.Publisher('zone', Marker, queue_size=10)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.init_node('zone_publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        if(active_zoneset_config):
            for idx, zoneset in enumerate(active_zoneset_config.zonesets):
    
                #poly.polygon.points.append(Point32(1,1,0))
                range_info = 'min:{:+} max:{:+}'.format(zoneset.speed_lower, zoneset.speed_upper)
                pub.publish(get_marker('zoneset[' + str(idx) + "] safety " + range_info, zoneset.safety.points, 0, color=ColorRGBA(1,0,0,1)))
                pub.publish(get_marker('zoneset[' + str(idx) + "] warn " + range_info, zoneset.warn.points, 1, ColorRGBA(1,1,0,1),z_offset=-0.01))
                rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber("/config_server_node/zonesets", ZoneSetConfiguration, zoneset_configuration_callback)
        talker()
    except rospy.ROSInterruptException:
        pass