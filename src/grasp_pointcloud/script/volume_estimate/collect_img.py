#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import os


# 回调函数，保存点云和图片
def callback(name):
    # 相机输入的图像数据
    rgb = rospy.wait_for_message("/camera/color/image_raw", Image)
    depth = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
    # 图像格式转换及保存
    bridge = CvBridge()
    try:
        rgb_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
        depth_img = bridge.imgmsg_to_cv2(depth, "64FC1")
        depth_img = np.round(depth_img / 2)
        path = "/home/jay/grasp_ws/src/grasp_pointcloud/script/volume_estimate/"
        if not os.path.exists(path):
            os.makedirs(path)
        cv.imwrite(path + "rgb_img/" + str(name) + ".png", rgb_img)
        cv.imwrite(path + "depth_img/" + str(name) + ".png", depth_img)
        print()
    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    # 初始化ros节点
    rospy.init_node("collect_img")
    rospy.loginfo("图像采集程序开始")
    sub = rospy.Subscriber("collect/collectable", String, callback, queue_size=1, buff_size=52428800)
    rospy.spin()