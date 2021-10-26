#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from psen_scan_v2.msg import ZoneSetConfiguration

active_zoneset_config = None

def zoneset_configuration_callback(zoneset_config):
    global active_zoneset_config
    active_zoneset_config = zoneset_config

def get_marker(ns, points, id, color, frame_id, z_offset=0):
    global speed_scale
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = id
    marker.type = marker.TRIANGLE_LIST
    marker.action = marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 0.4

    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = z_offset

    for p1,p2 in zip(points, points[1:]):
        marker.points.append(Point(0,0,0))
        marker.points.append(Point(p1.x,p1.y,0))
        marker.points.append(Point(p2.x,p2.y,0))
        marker.colors.append(color)
    return marker

def publish_zoneconfiguration_visualizations(pub):
    global active_zoneset_config

    if(active_zoneset_config):
        for idx, zoneset in enumerate(active_zoneset_config.zonesets):

            range_info = ""
            if zoneset.speed_lower != 0 or zoneset.speed_upper != 0:
                range_info = 'min:{:+} max:{:+}'.format(zoneset.speed_lower, zoneset.speed_upper)
            if len(zoneset.safety1.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] safety1 " + range_info, zoneset.safety1.points, 0, ColorRGBA(1,0,0,1), zoneset.header.frame_id))
            if len(zoneset.safety2.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] safety2 " + range_info, zoneset.safety2.points, 0, ColorRGBA(1,0,0,1),zoneset.header.frame_id))
            if len(zoneset.safety3.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] safety3 " + range_info, zoneset.safety3.points, 0, ColorRGBA(1,0,0,1),zoneset.header.frame_id))
            if len(zoneset.warn1.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] warn1 " + range_info, zoneset.warn1.points, 1, ColorRGBA(1,1,0,1),zoneset.header.frame_id, z_offset=0.01))
            if len(zoneset.warn2.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] warn2 " + range_info, zoneset.warn2.points, 1, ColorRGBA(1,1,0,1),zoneset.header.frame_id, z_offset=0.01))
            if len(zoneset.muting1.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] muting1 " + range_info, zoneset.muting1.points, 1, ColorRGBA(0,0,1,1),zoneset.header.frame_id, z_offset=0.02))
            if len(zoneset.muting2.points):
                pub.publish(get_marker('zoneset[' + str(idx) + "] muting2 " + range_info, zoneset.muting2.points, 1, ColorRGBA(0,0,1,1),zoneset.header.frame_id, z_offset=0.02))


if __name__ == '__main__':
    try:
        rospy.init_node('zoneconfiguration_visualization_node', anonymous=True)
        rospy.Subscriber(rospy.get_namespace() + "zoneconfiguration", ZoneSetConfiguration, zoneset_configuration_callback)
        pub = rospy.Publisher('zoneconfiguration_visualisation', Marker, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            publish_zoneconfiguration_visualizations(pub)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass