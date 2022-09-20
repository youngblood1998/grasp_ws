#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import copy
import sys

from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from real_grasp_tree_build_new_1 import grasp_tree_builder
from real_grasp_candidate_generate_new_1 import grasp_candidate_generator
from real_grasp_pose_evaluate_new_1 import grasp_pose_evaluator
from grasp_pointcloud.msg import GraspParams


class GraspDetector:
    def __init__(self):
        self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.bound_sub = message_filters.Subscriber("/yolov5/BoundingBoxes", BoundingBoxes, queue_size=1, buff_size=52428800)
        self.tree_pub = rospy.Publisher("real_detect/tree_image", Image, queue_size=1)
        self.rgb_cut_pub = rospy.Publisher("real_detect/rgb_cut_image", Image, queue_size=1)
        self.result_pub = rospy.Publisher("real_detect/result_image", Image, queue_size=1)
        self.grasp_params_pub = rospy.Publisher("real_detect/grasp_params", GraspParams, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.bound_sub], 1, 0.4, allow_headerless=True)
        sync.registerCallback(self.call_back)
        # 用于在保存连续多帧话题
        self.topic_arr = []

    def call_back(self, color_img, depth_img, bound):
        #如果参数grasp_step不存在或者不等于0说明机器人在移动,暂停检测
        if rospy.has_param("/grasp_step") and int(rospy.get_param("/grasp_step"))!=0:
            return 0
        bridge = CvBridge()
        try:
            # 话题
            color_img = bridge.imgmsg_to_cv2(color_img, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(depth_img, "64FC1")
            # 将深度图处理成64位整形
            depth_img = np.round(depth_img).astype(np.int16)
            # 始终维持数组长度为3
            topic = [color_img, depth_img, bound]
            self.topic_arr.append(copy.deepcopy(topic))
            if len(self.topic_arr) > 3:
                del self.topic_arr[0]
            # 连续几帧中取锚框最少的
            best_topic = None
            num = sys.maxint
            for t in self.topic_arr:
                if len(t[2].bounding_boxes) < num:
                    best_topic = t
                    num = len(t[2].bounding_boxes)
            # 没有则返回
            if best_topic is None:
                return
            # 最佳那一帧的深度图、彩色图以及锚框
            best_depth_img = best_topic[1]
            best_color_img = best_topic[0]
            best_bound = best_topic[2].bounding_boxes
            # 树建立并获取抓取对象的xmin, xmax, ymin, ymax, mean_depth
            bound_data, tree_img = grasp_tree_builder(best_depth_img, best_color_img, best_bound)
            if bound_data is None:
                return
            self.tree_pub.publish(bridge.cv2_to_imgmsg(tree_img, "bgr8"))
            # 截取图片
            rgb_cut = best_color_img[int(bound_data[2]):int(bound_data[3]), int(bound_data[0]):int(bound_data[1])]
            self.rgb_cut_pub.publish(bridge.cv2_to_imgmsg(rgb_cut, "bgr8"))
            # 抓取候选生成
            depth_img_cut, line_arr, cut_img_min_point = grasp_candidate_generator(best_depth_img, best_color_img, bound_data)
            # 评估抓取候选选择最优
            grasp_point, rotate_angle, tilt_angle, grasp_width_first, grasp_width_second, result_img = grasp_pose_evaluator(bound_data, best_depth_img, best_color_img, depth_img_cut, line_arr, cut_img_min_point)
            if len(grasp_point) == 0:
                return 
            self.result_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
            # 发布抓取参数
            grasp_params = GraspParams()
            grasp_params.x = grasp_point[0]/1000.0
            grasp_params.y = grasp_point[1]/1000.0
            grasp_params.z = grasp_point[2]/1000.0
            grasp_params.rotate_angle = rotate_angle
            grasp_params.tilt_angle = tilt_angle
            grasp_params.grasp_width_first = grasp_width_first
            grasp_params.grasp_width_second = grasp_width_second
            print(grasp_params)
            self.grasp_params_pub.publish(grasp_params)
        except CvBridgeError as e:
            print("CvBridge转换出错！！！")


if __name__ == "__main__":
    rospy.init_node("grasp_detector")
    rospy.loginfo("抓取检测开始")
    GraspDetector()
    rospy.spin()
