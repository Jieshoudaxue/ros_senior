#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from functools import partial
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest

class DetectorParam:
    def __init__(self):
      self.minArea = rospy.get_param("~minArea",   500)
      self.threshold = rospy.get_param("~threshold", 25)
      self.firstFrame = None
      self.text = "Unoccupied"

def image_cb(msg, cv_bridge, detector_param, image_pub):
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = np.array(cv_image, dtype=np.uint8)
    except (CvBridgeError, e):
        print(e)
        
    # 创建灰度图像
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("ycao: gray image", gray_image)
    cv2.waitKey(3)        
    gray_image = cv2.GaussianBlur(gray_image, (21,21), 0)
    cv2.imshow("ycao: Gaussian blur image", gray_image)
    cv2.waitKey(3)     
    
    # 使用两帧图像做比较，检测移动物体的区域
    if detector_param.firstFrame is None:
      detector_param.firstFrame = gray_image
      return
      
    frameDelta = cv2.absdiff(detector_param.firstFrame, gray_image)
    threshold = cv2.threshold(frameDelta, detector_param.threshold, 255, cv2.THRESH_BINARY)[1]
    cv2.imshow("ycao: THRESH BINARY image", threshold)
    cv2.waitKey(3)    
    threshold = cv2.dilate(threshold, None, iterations=2)
    cv2.imshow("ycao: dilate image", threshold)
    cv2.waitKey(3)    
    cnts, hierarchy = cv2.findContours(threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
      # 如果检测到的区域小于设置值，则忽略
      if cv2.contourArea(c) < detector_param.minArea:
        continue
      
      # 在输出画面上框出识别到的物体
      (x,y,w,h) = cv2.boundingRect(c)
      cv2.rectangle(frame, (x,y), (x+w, y+h), (50,255,50), 2)
      detector_param.text = "Occupied"  
      
    # 在输出画面上打当前状态和时间戳信息
    cv2.putText(frame, "Status: {}".format(detector_param.text), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    
    # 将识别后的图像转换成ROS消息并发布
    image_pub.publish(cv_bridge.cv2_to_imgmsg(frame, "bgr8"))    
    
def main():
    rospy.init_node("motion_detector")
    rospy.loginfo("starting motion_detector node")

    bridge = CvBridge()
    image_pub = rospy.Publisher("/cv_bridge_image", Image, queue_size=1)

    detector_param = DetectorParam()
    
    bind_image_cb = partial(image_cb, cv_bridge=bridge, detector_param=detector_param, image_pub=image_pub)

    rospy.Subscriber("/usb_cam/image_raw", Image, bind_image_cb)

    
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()