#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import sys
import numpy as np

class DynamicZoom:
    def __init__(self):
        rospy.init_node('dynamic_zoom_node', anonymous=True)

        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            rospy.logerr("It is necessary to specify the name of the image topic and the zoom factor topic as arguments to the executable. For example: /camera/image_raw /factor_zoom")
            sys.exit(1)
        self.sub_image = argv[1]
        self.sub_factor_zoom = argv[2]

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.sub_image, Image, self.image_callback)
        self.zoom_factor_sub = rospy.Subscriber(self.sub_factor_zoom, Float64, self.zoom_factor_callback)
        self.zoom_factor = 1.0 
        self.zoomed_image_pub = rospy.Publisher("/camera/zoom/image_raw", Image, queue_size=10)

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = cv_image.shape

        zoom_percentage = 1.0 - self.zoom_factor / 100.0
        new_width = int(width * zoom_percentage)
        new_height = int(height * zoom_percentage)

        start_x = int((width - new_width) / 2)
        start_y = int((height - new_height) / 2)

        zoomed_image = cv_image[start_y:start_y+new_height, start_x:start_x+new_width]
        
        zoomed_image_msg = self.bridge.cv2_to_imgmsg(zoomed_image, "bgr8")
        self.zoomed_image_pub.publish(zoomed_image_msg)

    def zoom_factor_callback(self, data):
        if data.data < 0.0:
            self.zoom_factor = 0.0
        elif data.data >= 100.0:
            self.zoom_factor = 99.0
        else:
            self.zoom_factor = data.data

if __name__ == '__main__':
    dynamic_zoom = DynamicZoom()
    rospy.spin()
