#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from functools import partial
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest


def image_cb(msg, cv_bridge, image_pub):
    pass
    
def main():
    rospy.init_node("face_detector")
    rospy.loginfo("starting face_detector node")

    bridge = CvBridge()
    image_pub = rospy.Publisher("/cv_bridge_image", Image, queue_size=1)
    
    bind_image_cb = partial(image_cb, cv_bridge=bridge, image_pub=image_pub)

    rospy.Subscriber("/usb_cam/image_raw", Image, bind_image_cb)

    
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()