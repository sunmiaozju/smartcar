#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
import cv_bridge
from image_view2.msg import MouseEvent


def main():
    pub_plus = rospy.Publisher('~plus_rect_event', MouseEvent, queue_size=1)
    pub_minus = rospy.Publisher('~minus_rect_event', MouseEvent, queue_size=1)

    width = rospy.get_param('~image_width')
    height = rospy.get_param('~image_height')
    plus_events = [
        MouseEvent(type=3, x=width/4, y=height/4, width=width, height=height),
        MouseEvent(type=4, x=width/2, y=height/2, width=width, height=height),
        MouseEvent(type=2, x=3*width/4, y=3*height/4, width=width, height=height),
    ]
    minus_events = [
        MouseEvent(type=3, x=3*width/4, y=3*height/4, width=width, height=height),
        MouseEvent(type=4, x=width/2, y=height/2, width=width, height=height),
        MouseEvent(type=2, x=width/4, y=height/4, width=width, height=height),
    ]
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for e in plus_events:
            e.header.stamp = rospy.get_rostime()
            pub_plus.publish(e)
            rate.sleep()
        for e in minus_events:
            e.header.stamp = rospy.get_rostime()
            pub_minus.publish(e)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('publish_mouse_event')
    main()
