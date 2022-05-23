#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import copy

from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from real_grasp_tree_build import tree_built
from real_grasp_dbscan import dbscan


class GraspDetector:
    def __init__(self):
        # self.color_sub = message_filters.Subscriber("/yolov5/detection_image", Image, queue_size=1, buff_size=52428800)
        self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.bound_sub = message_filters.Subscriber("/yolov5/BoundingBoxes", BoundingBoxes, queue_size=1, buff_size=52428800)
        self.img_pub = rospy.Publisher("real_detect/tree_image", Image, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.bound_sub], 1, 0.4, allow_headerless=True)
        sync.registerCallback(self.call_back)

        # 用于在保存连续多帧话题
        self.topic_arr = []

    def call_back(self, color_img, depth_img, bound):
        bridge = CvBridge()
        try:
            # 话题
            color_img = bridge.imgmsg_to_cv2(color_img, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(depth_img, "64FC1")
            # 将深度图处理成64位整形
            depth_img = np.round(depth_img).astype(np.int64)

            # print(len(bound.bounding_boxes))

            # 始终维持数组长度为3
            topic = [color_img, depth_img, bound]
            self.topic_arr.append(copy.deepcopy(topic))
            if len(self.topic_arr) > 3:
                del self.topic_arr[0]
            # 连续几帧中取锚框最多的
            best_topic = None
            num = 0
            for t in self.topic_arr:
                if len(t[2].bounding_boxes) > num:
                    best_topic = t
                    num = len(t[2].bounding_boxes)
            # 没有则返回
            if best_topic is None:
                return
            # 最佳那一帧的深度图、彩色图以及锚框
            best_depth_img = best_topic[1]
            best_color_img = best_topic[0]
            best_bound = best_topic[2]

            # 数结构选取抓取对象
            result_img, bound_data = tree_built(best_depth_img, best_color_img, best_bound)
            # 发布带有树结构的彩色图
            self.img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
            print(0)
            # 对三维点进行DBSCAN聚类
            dbscan(best_depth_img, bound_data)
            print(1)
        except CvBridgeError as e:
            print("CvBridge转换出错！！！")

if __name__ == "__main__":
    rospy.init_node("grasp_detector")
    rospy.loginfo("抓取检测开始")
    GraspDetector()
    rospy.spin()
