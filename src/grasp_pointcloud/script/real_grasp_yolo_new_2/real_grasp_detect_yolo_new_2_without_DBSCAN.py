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
from real_grasp_tree_build_new_2 import grasp_tree_builder
from real_grasp_candidate_generate_new_2_without import grasp_candidate_generator
from real_grasp_pose_evaluate_new_2 import grasp_pose_evaluator
from grasp_pointcloud.msg import GraspParams

MAX_NODE_NUM = 3

class GraspDetector:
    def __init__(self):
        # 彩色图和深度图以及yolov5检测信息的订阅者
        self.color_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.bound_sub = message_filters.Subscriber("/yolov5/BoundingBoxes", BoundingBoxes, queue_size=1, buff_size=52428800)
        # 树形图，彩色剪切图，抓取结果图以及抓取参数的发布者
        self.tree_pub = rospy.Publisher("real_detect/tree_image", Image, queue_size=1)
        self.rgb_cut_pub = rospy.Publisher("real_detect/rgb_cut_image", Image, queue_size=1)
        self.result_pub = rospy.Publisher("real_detect/result_image", Image, queue_size=1)
        self.grasp_params_pub = rospy.Publisher("real_detect/grasp_params", GraspParams, queue_size=1)
        # 多话题同步
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
            # cvbridge转换imgmsg为cv类型
            color_img = bridge.imgmsg_to_cv2(color_img, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(depth_img, "64FC1")
            # 将深度图处理成64位整形并进行深度滤波
            depth_img = np.round(depth_img).astype(np.int16)
            depth_img = cv2.medianBlur(depth_img, 5)
            # # 始终维持数组长度为3
            # topic = [color_img, depth_img, bound]
            # self.topic_arr.append(copy.deepcopy(topic))
            # if len(self.topic_arr) > 3:
            #     del self.topic_arr[0]
            # # 连续几帧中取锚框最少的
            # best_topic = None
            # num = sys.maxint
            # for t in self.topic_arr:
            #     if len(t[2].bounding_boxes) < num:
            #         best_topic = t
            #         num = len(t[2].bounding_boxes)
            # # 没有话题则返回
            # if best_topic is None:
            #     return
            # # 最佳那一帧的深度图、彩色图以及锚框
            # best_depth_img = best_topic[1]
            # best_color_img = best_topic[0]
            # best_bound = best_topic[2].bounding_boxes
            # 树建立并获取抓取对象的xmin, xmax, ymin, ymax, mean_depth
            node_arr, tree_img = grasp_tree_builder(depth_img, color_img, bound.bounding_boxes)
            self.tree_pub.publish(bridge.cv2_to_imgmsg(tree_img, "bgr8"))
            # 对第一层节点进行计算
            i = 0
            max_area = 0
            best_grasp_datas = None
            best_result_img = []
            while i < MAX_NODE_NUM and i < len(node_arr):
                node = node_arr[i]
                i += 1
                # 截取图片
                rgb_cut = color_img[int(node.data[2]):int(node.data[3]), int(node.data[0]):int(node.data[1])]
                # self.rgb_cut_pub.publish(bridge.cv2_to_imgmsg(rgb_cut, "bgr8"))
                # 抓取候选生成
                depth_img_cut, line_arr, cut_img_min_point = grasp_candidate_generator(depth_img, color_img, node.data)
                # 评估抓取候选选择最优
                area, grasp_datas, result_img = grasp_pose_evaluator(node, depth_img, color_img, depth_img_cut, line_arr, cut_img_min_point)
                if area > max_area:
                    best_grasp_datas = grasp_datas
                    best_result_img = result_img
                    max_area = area
            if max_area == 0:
                return
            # # 对第一层节点进行计算
            # i = 0
            # max_area_width_ratio = 0
            # best_grasp_datas = None
            # best_result_img = []
            # while i < MAX_NODE_NUM and i < len(node_arr):
            #     node = node_arr[i]
            #     i += 1
            #     # 截取图片
            #     rgb_cut = color_img[int(node.data[2]):int(node.data[3]), int(node.data[0]):int(node.data[1])]
            #     # self.rgb_cut_pub.publish(bridge.cv2_to_imgmsg(rgb_cut, "bgr8"))
            #     # 抓取候选生成
            #     depth_img_cut, line_arr, cut_img_min_point = grasp_candidate_generator(depth_img, color_img, node.data)
            #     # 评估抓取候选选择最优
            #     area, grasp_datas, result_img = grasp_pose_evaluator(node, depth_img, color_img, depth_img_cut, line_arr, cut_img_min_point)
            #     if area / grasp_datas[4] > max_area_width_ratio:
            #         best_grasp_datas = grasp_datas
            #         best_result_img = result_img
            #         max_area_width_ratio = area / grasp_datas[4]
            #     print(area, grasp_datas[4])
            #     print("抓取深度宽度比")
            #     print(max_area_width_ratio)
            # if max_area_width_ratio == 0:
            #     return
            self.result_pub.publish(bridge.cv2_to_imgmsg(best_result_img, "bgr8"))
            # 发布抓取参数
            grasp_point = best_grasp_datas[0]
            grasp_params = GraspParams()
            grasp_params.x = grasp_point[0]/1000.0
            grasp_params.y = grasp_point[1]/1000.0
            grasp_params.z = grasp_point[2]/1000.0
            grasp_params.rotate_angle = best_grasp_datas[1]
            grasp_params.tilt_angle = best_grasp_datas[2]
            grasp_params.grasp_width_first = best_grasp_datas[3]
            grasp_params.grasp_width_second = best_grasp_datas[4]
            # print(grasp_params)
            self.grasp_params_pub.publish(grasp_params)
        # except CvBridgeError as e:
        #     print("CvBridge转换出错！！！")
        except Exception as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("grasp_detector")
    rospy.loginfo("抓取检测开始")
    GraspDetector()
    rospy.spin()
