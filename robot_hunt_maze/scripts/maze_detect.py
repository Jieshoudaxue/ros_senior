#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from functools import partial
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import numpy as np

BLUE_LOW   = 0
BLUE_HIGH  = 20
GREEN_LOW  = 20
GREEN_HIGH = 60
RED_LOW    = 80
RED_HIGH   = 150

def image_cb(msg, cv_bridge, image_pub, maze_pose_pub):
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # define the list of boundaries in BGR
    boundaries = [([BLUE_LOW, GREEN_LOW, RED_LOW], [BLUE_HIGH, GREEN_HIGH, RED_HIGH])]
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply the mask
    mask = cv2.inRange(cv_image, lower, upper)
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    cvImg = cv2.cvtColor(output, 6)
    npImg = np.asarray( cvImg )
    thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

    # find contours in the thresholded image
    cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        M = cv2.moments(c)
        
        if int(M["m00"]) not in range(5000, 250000):
            continue

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # print("cX {0} cY {1} m00 {2}".format(cX, cY, int(M["m00"])))

        cv2.drawContours(cv_image, [c], -1, (0, 0, 255), 2)
        cv2.circle(cv_image, (cX, cY), 1, (0, 0, 255), -1)
        objPose = Pose()
        objPose.position.x = cX
        objPose.position.y = cY
        objPose.position.z = M["m00"]
        maze_pose_pub.publish(objPose)

    # 再将opencv格式额数据转换成ros image格式的数据发布
    try:
        image_pub.publish(cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)

    
def main():
    rospy.init_node("maze_detect")
    rospy.loginfo("starting maze_detect.py node")

    bridge = CvBridge()
    image_pub = rospy.Publisher("/maze_image", Image, queue_size=1)
    maze_pose_pub = rospy.Publisher("/maze_pose", Pose, queue_size=1)
    
    bind_image_cb = partial(image_cb, cv_bridge=bridge, image_pub=image_pub, maze_pose_pub=maze_pose_pub)

    rospy.Subscriber("/camera_front/image_raw", Image, bind_image_cb)
    rospy.Subscriber("/camera_rear/image_raw", Image, bind_image_cb)

    
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()