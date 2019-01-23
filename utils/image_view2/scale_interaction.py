#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Polygon, Point32, PolygonStamped
from std_msgs.msg import Float32
from threading import Lock

width_scale = None
height_scale = None
pending_polygon = None
lock = Lock()

def widthScallCallback(msg):
    global width_scale
    with lock:
        if msg.data == 0:
            rospy.logdebug("scale is 0!, we skip it")
        else:
            width_scale = msg.data
        updateForPendingData()
    
def heightScallCallback(msg):
    global height_scale
    with lock:
        if msg.data == 0:
            rospy.logdebug("scale is 0!, we skip it")
        else:
            height_scale = msg.data
        updateForPendingData()
    
def polygonCallback(msg):
    global pending_polygon
    with lock:
        if width_scale and height_scale:
            publishResizedPolygon(msg)
        else:
            pending_polygon = msg

def updateForPendingData():
    global pending_polygon, width_scale, height_scale
    if (pending_polygon and width_scale and height_scale
        and width_scale != 0 and height_scale != 0):
        publishResizedPolygon(pending_polygon)
        pending_polygon = None
        
def publishResizedPolygon(poly_msg):
    global width_scale, height_scale, inverse, pub
    if inverse:
        Rw = 1.0 / width_scale
        Rh = 1.0 / height_scale
    else:
        Rw = width_scale
        Rh = height_scale
    for point in poly_msg.polygon.points:
        point.x = Rw * point.x
        point.y = Rh * point.y
    pub.publish(poly_msg)

rospy.init_node("scale_interaction")

inverse = rospy.get_param("~inverse", False)
pub = rospy.Publisher("~output", PolygonStamped)
sub_polygon = rospy.Subscriber("~input", PolygonStamped, polygonCallback)
sub_width = rospy.Subscriber("~input/width_scale", Float32, widthScallCallback)
sub_height = rospy.Subscriber("~input/height_scale", Float32, heightScallCallback)

rospy.spin()
