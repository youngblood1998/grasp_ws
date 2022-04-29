#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from real_grasp_tree_build import tree_built


OVERLAP_RATIO = 0.2 # 重叠占比

class GraspDetector:
    def __init__(self):
        self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.bound_sub = message_filters.Subscriber("/yolov5/BoundingBoxes", BoundingBoxes, queue_size=1, buff_size=52428800)
        sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.bound_sub], 1, 0.4, allow_headerless=True)
        sync.registerCallback(self.call_back)

    def call_back(self, color_img, depth_img, bound):
        bridge = CvBridge()
        try:
            color_img = bridge.imgmsg_to_cv2(color_img, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(depth_img, "64FC1")
            # 将深度图处理成64位整形
            depth_img = np.round(depth_img).astype(np.int64)
            # 数结构选取抓取对象
            tree_built(depth_img, bound, OVERLAP_RATIO)
        except CvBridgeError as e:
            print("CvBridge转换出错！！！")

if __name__ == "__main__":
    rospy.init_node("grasp_detector")
    rospy.loginfo("抓取检测开始")
    GraspDetector()
    rospy.spin()
