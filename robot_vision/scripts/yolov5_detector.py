#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import torch
import numpy as np
from functools import partial
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, RegionOfInterest
from robot_vision.msg import BoundingBox, BoundingBoxes

class Yolov5Param:
    def __init__(self):
        # load local repository(YoloV5:v6.0)
        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_path = rospy.get_param('~weight_path', '')
        conf = rospy.get_param('~conf', '0.5')
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()
        self.model.conf = conf

        # target publishers
        self.target_pub = rospy.Publisher("/yolov5/targets",  BoundingBoxes, queue_size=1)

def image_cb(msg, cv_bridge, yolov5_param, color_classes, image_pub):
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = np.array(cv_image, dtype=np.uint8)
    except (CvBridgeError, e):
        print(e)
    
    bounding_boxes = BoundingBoxes()
    bounding_boxes.header = msg.header

    # 将BGR图像转换为RGB图像, 给yolov5
    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = yolov5_param.model(rgb_image)
    
    boxs = results.pandas().xyxy[0].values
    for box in boxs:
        bounding_box = BoundingBox()
        bounding_box.probability =np.float64(box[4])
        bounding_box.xmin = np.int64(box[0])
        bounding_box.ymin = np.int64(box[1])
        bounding_box.xmax = np.int64(box[2])
        bounding_box.ymax = np.int64(box[3])
        bounding_box.num = np.int16(len(boxs))
        bounding_box.Class = box[-1]
        
        # 放入box队列中
        bounding_boxes.bounding_boxes.append(bounding_box)
        
        if box[-1] in color_classes.keys():
            color = color_classes[box[-1]]
        else:
            color = np.random.randint(0, 183, 3)
            color_classes[box[-1]] = color
    
        # 用框把目标圈出来
        cv2.rectangle(cv_image, (int(box[0]), int(box[1])),
                        (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)    

        if box[1] < 20:
            text_pos_y = box[1] + 30
        else:
            text_pos_y = box[1] - 10    
        
        # 在框上, 打印物体类型信息Class
        cv2.putText(cv_image, box[-1],
                    (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)    
    
    
    # 发布目标数据
    yolov5_param.target_pub.publish(bounding_boxes)
    
    # 将识别后的图像转换成ROS消息并发布
    image_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main():
    rospy.init_node("yolov5_detector")
    rospy.loginfo("starting yolov5_detector node")

    bridge = CvBridge()
    image_pub = rospy.Publisher("/yolov5/detection_image", Image, queue_size=1)
    
    yolov5_param = Yolov5Param()
    color_classes = {}
    
    bind_image_cb = partial(image_cb, cv_bridge=bridge, yolov5_param=yolov5_param, color_classes=color_classes, image_pub=image_pub)

    rospy.Subscriber("/usb_cam/image_raw", Image, bind_image_cb)

    
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()