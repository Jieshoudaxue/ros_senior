#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from functools import partial
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Twist

class HaarParam:
    def __init__(self):
        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")
        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)

def detect_face(input_image, haar_param):
    # 首先匹配正面人脸的模型
    if haar_param.cascade_1:
        faces = haar_param.cascade_1.detectMultiScale(input_image,
                                                    haar_param.haar_scaleFactor,
                                                    haar_param.haar_minNeighbors,
                                                    cv2.CASCADE_SCALE_IMAGE,
                                                    (haar_param.haar_minSize, haar_param.haar_maxSize))

    # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
    if len(faces) == 0 and haar_param.cascade_2:
        faces = haar_param.cascade_2.detectMultiScale(input_image,
                                                    haar_param.haar_scaleFactor,
                                                    haar_param.haar_minNeighbors,
                                                    cv2.CASCADE_SCALE_IMAGE,
                                                    (haar_param.haar_minSize, haar_param.haar_maxSize))
    return faces

def vel_control(x, y, w, h):
    speed = 0.5
    turn = 0.5
    line_x = 0
    th = 0
    control_speed = 0
    control_turn = 0
    rospy.loginfo("x %d, y %d, w %d, h %d" %(x,y,w,h))
    # face turn right 
    if x <= 200:
        th = -1
    # face turn left
    elif x > 320:
        th = 1
    if w < 220:
        line_x = -1
    elif w > 280:
        line_x = 1
    
    control_speed = speed * line_x
    control_turn = turn * th
    return (control_speed, control_turn)

def image_cb(msg, cv_bridge, haar_param, image_pub, cmd_pub):
    control_speed = 0
    control_turn = 0

    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = np.array(cv_image, dtype=np.uint8)
    except (CvBridgeError, e):
        print(e)
    
    # 创建灰度图像
    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("ycao: grey image", grey_image)
    cv2.waitKey(3)

    # 创建平衡直方图，减少光线影响
    grey_image = cv2.equalizeHist(grey_image)
    cv2.imshow("ycao: histogram equalization image", grey_image)
    cv2.waitKey(3)

    # 尝试检测人脸
    faces_result = detect_face(grey_image, haar_param)
    
    # 在opencv的窗口中框出所有人脸区域
    if len(faces_result) > 0:
        for face in faces_result:
            x,y,w,h = face
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), haar_param.color, 2)

            control_speed,control_turn = vel_control(x,y,w,h)
            
            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = control_speed; 
            twist.linear.y = 0; 
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = control_turn
            cmd_pub.publish(twist)    
    else:
        print("%u: no face in current image" %rospy.get_time())
    
    # 将识别后的图像转换成ROS消息并发布
    image_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))


    
def main():
    rospy.init_node("face_detector")
    rospy.loginfo("starting face_detector node")

    bridge = CvBridge()
    image_pub = rospy.Publisher("/cv_bridge_image", Image, queue_size=1)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    
    haar_param = HaarParam()
    
    bind_image_cb = partial(image_cb, cv_bridge=bridge, haar_param=haar_param, image_pub=image_pub, cmd_pub=cmd_pub)

    rospy.Subscriber("/usb_cam/image_raw", Image, bind_image_cb)

    
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()