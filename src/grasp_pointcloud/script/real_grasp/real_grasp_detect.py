#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float32MultiArray
from math import pi

from real_grasp_candidate_generator import depth_filter_new, dbscan, grasp_generation_new
from real_grasp_pose_evaluator import gmm, grasp_show


# INIT_THRESH = 400       #深度滤波的初始值
THRESH_STEP = 4         #深度滤波的步长
R_VAL = 10              #dbscan的r值
MIN_NUM = 5             #dbscan的min_num值
LINE_NUM = 10           #每个类生成的线数
T_VAL = 3/2             #生成的矩形和线段长度的倍数
GRIPPER_HEIGHT = 6      #夹爪的厚度
GRIPPER_WIDTH = 23     #夹爪宽度
ADD_WIDTH = 23          #抓取余量

class Image_converter:
    def __init__(self):
        #接受深度图、彩色图后调用回调函数
        self.rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
        self.cluster_pub = rospy.Publisher("/img/cluster_img", Image, queue_size=1)
        self.result_pub = rospy.Publisher("/img/result_img", Image, queue_size=1)
        self.point_pub = rospy.Publisher("/point/grasp_point", Float32MultiArray, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 1, 0.005, allow_headerless=True)
        sync.registerCallback(self.call_back)

    def call_back(self, rgb, depth):
        #如果参数grasp_step不存在或者不等于0说明在抓取,暂停检测
        if rospy.has_param("/grasp_step") and int(rospy.get_param("/grasp_step")) != 0:
            print("正在抓取")
            return 0
        #检测
        bridge = CvBridge()
        try:
            rgb_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(depth, "64FC1")
            #将深度图处理成64位整形
            depth_img = np.round(depth_img).astype(np.int64)
            np.set_printoptions(threshold=np.inf)
            #从上往下深度滤波
            # thresh = INIT_THRESH
            thresh = sorted(np.unique(depth_img))[1]
            binary, depth, max_val = depth_filter_new(depth_img, thresh)
            cluster_img, cluster_arr = dbscan(binary, R_VAL, MIN_NUM)
            while len(cluster_img) == 0:
                thresh += THRESH_STEP
                print("深度滤波值:{}毫米".format(thresh))
                binary, depth, max_val = depth_filter_new(depth_img, thresh)
                cluster_img, cluster_arr = dbscan(binary, R_VAL, MIN_NUM)
            #发布二值图和聚类图
            # self.binary_pub.publish(bridge.cv2_to_imgmsg(binary, "64UC1"))
            self.cluster_pub.publish(bridge.cv2_to_imgmsg(cluster_img, "bgr8"))
            rect_arr, cent_arr, lines_arr = grasp_generation_new(cluster_arr, cluster_img, LINE_NUM, T_VAL)
            #评估每个抓取
            index_arr = []
            lr_arr = []
            depth_arr = []
            width_arr = []
            add_width_arr = []
            for i, rect in enumerate(rect_arr):
                slice_img = depth[rect[0]:rect[1], rect[2]:rect[3]]
                select_index, lr_index, select_depth, select_width, select_add_width = gmm(slice_img, lines_arr[i], GRIPPER_HEIGHT, LINE_NUM, max_val, ADD_WIDTH)
                index_arr.append(select_index)
                lr_arr.append(lr_index)
                depth_arr.append(select_depth)
                width_arr.append(select_width)
                add_width_arr.append(select_add_width)
            point, result_img = grasp_show(rgb_img, index_arr, lr_arr, cent_arr, rect_arr, depth_arr, LINE_NUM)
            self.result_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
            #将消息转换为数组类型并发布
            pub_point = []
            for i, p in enumerate(point):
                p = [r/1000 for r in p]
                pub_point.extend(p)
                pub_point.append(index_arr[i]*pi/LINE_NUM)
                pub_point.append(width_arr[i])
                pub_point.append(add_width_arr[i])
            pub_point = Float32MultiArray(data=pub_point)
            self.point_pub.publish(pub_point)
        except CvBridgeError as e:
            print("CvBridge转换出错！！！")

if __name__ == "__main__":
    rospy.init_node("grasp_pose_detect")
    rospy.loginfo("抓取检测开始")
    Image_converter()
    rospy.spin()