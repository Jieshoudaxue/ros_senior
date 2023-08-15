#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from functools import partial
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def image_cb(msg, cv_bridge, image_pub):
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # 在opencv的显示窗口中绘制一个圆，作为标记
    (rows, cols, channels) = cv_image.shape
    if cols > 60 and rows > 60:
        cv2.circle(cv_image, (60,60), 30, (255,0,0), -1)


    # 显示Opencv格式的图像
    cv2.imshow("ycao: opencv image window", cv_image)
    cv2.waitKey(3)

    # 再将opencv格式额数据转换成ros image格式的数据发布
    try:
        image_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)

    
def main():
    rospy.init_node("cv_bridge_test")
    rospy.loginfo("starting cv_bridge_test node")

    bridge = CvBridge()
    image_pub = rospy.Publisher("/cv_bridge_image", Image, queue_size=1)
    
    bind_image_cb = partial(image_cb, cv_bridge=bridge, image_pub=image_pub)

    rospy.Subscriber("/usb_cam/image_raw", Image, bind_image_cb)

    
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()