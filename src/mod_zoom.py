#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import sys

class DynamicZoom:
    def __init__(self):
        rospy.init_node('dynamic_zoom_node', anonymous=True)

        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 3:
            rospy.logerr("It is necessary to specify the name of the image theme and the zoom factor as an argument to the executable. For example: /camera/image_raw /factor_zoom")
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
        
        height, width = cv_image.shape[:2]
        rospy.loginfo("Image size: " + str(width) + "x" + str(height))

        new_width = int(width / self.zoom_factor)
        new_height = int(height / self.zoom_factor)
        rospy.loginfo("New image size: " + str(new_width) + "x" + str(new_height))

        start_x = int((width - new_width) / 2)
        start_y = int((height - new_height) / 2)
        rospy.loginfo("Start x: " + str(start_x) + " Start y: " + str(start_y))

        zoomed_image = cv_image[start_y:start_y+new_height, start_x:start_x+new_width]
        rospy.loginfo("Recortar desde " + str(start_x) + " hasta " + str(start_x+new_width) + " y desde " + str(start_y) + " hasta " + str(start_y+new_height))
        
        zoomed_image_msg = self.bridge.cv2_to_imgmsg(zoomed_image, "bgr8")
        self.zoomed_image_pub.publish(zoomed_image_msg)

    def zoom_factor_callback(self, data):
        if data.data < 1.0:
            self.zoom_factor = 1.0
        elif data.data > 10.0:
            self.zoom_factor = 10.0
        else:
            self.zoom_factor = data.data

if __name__ == '__main__':
    dynamic_zoom = DynamicZoom()
    rospy.spin()