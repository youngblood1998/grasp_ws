#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
from math import sin, cos, pi

from random_color import ncolors
from trans_func import trans_img2real_length, distance


R = 10      # DBSCAN的r值
MIN_NUM = 5 # DBSCAN的min_num值
AREA = 300  # 聚类的类面积阈值
T = 0.8       # 生成抓取候选所需截图的大小比例
LINE_NUM = 10   # 生成的抓取候选个数
SIPPLEMENT_VALUE = 2**15-1    # 空洞填补值
CUT_RATIO = 0.95       # 截取图片靠近聚类点的权值，0～1


# 深度滤波
def depth_filter(depth_img, thresh):
    # 拷贝并对空洞进行处理
    depth = depth_img.copy()
    depth[depth == 0] = SIPPLEMENT_VALUE
    # print("最小值")
    # print(np.min(depth))
    # 二值化
    depth[depth < thresh] = 0
    depth[depth >= thresh] = 255
    ret, binary = cv.threshold(depth, 1, 255, cv.THRESH_BINARY_INV)

    return binary


# 聚类
def dbscan(binary, r, min_num, thresh):
    # 找出255的点进行DBSCAN
    y_idx, x_idx = np.where(binary==255)
    datas = np.vstack((x_idx, y_idx)).T
    clusters = DBSCAN(eps=r, min_samples=min_num).fit_predict(datas)
    np.set_printoptions(threshold=np.inf)
    # 画出聚类效果图
    cluster_img = np.zeros((binary.shape[0], binary.shape[1], 3), dtype=np.uint8)
    num = max(clusters)+1
    color = ncolors(num)
    cluster_arr = []
    for i in range(0, num):
        cluster_arr.append(datas[clusters == i])
    # 从后往前遍历去除不合理的聚类
    for i in range(num-1, -1, -1):
        x_min, x_max = np.min(cluster_arr[i][:, 0]), np.max(cluster_arr[i][:, 0])
        y_min, y_max = np.min(cluster_arr[i][:, 1]), np.max(cluster_arr[i][:, 1])
        img_height, img_width = binary.shape[0], binary.shape[1]
        cent_x = (x_min+x_max)/2
        cent_y = (y_min+y_max)/2
        if trans_img2real_length(thresh, x_max-x_min)*trans_img2real_length(thresh, y_max-y_min) < AREA or distance((cent_x, cent_y), (img_width/2, img_height/2)) > min(img_width/2, img_height/2):
            del cluster_arr[i]
    # 没有合适的类时返回None
    if len(cluster_arr) == 0:
        return None, None
    # 给类上色
    for i in range(0, len(clusters)):
        cluster_img[y_idx[i], x_idx[i], :] = color[clusters[i]]
    
    return cluster_img, cluster_arr


#抓取姿态生成
def grasp_generate(depth_img, rgb_img, line_num):
    rgb_img_copy = rgb_img.copy()
    #确定中心点和长度
    centroid_x, centroid_y, length = int(depth_img.shape[1]/2), int(depth_img.shape[0]/2), depth_img.shape[0]-2
    cv.circle(rgb_img_copy, (centroid_x, centroid_y), 2, (0, 0, 255), 2)
    # 生成抓取候选
    line_arr = []
    for i in range(0, line_num):
        min_line_x = int(centroid_x-(length/2)*cos(pi*i/line_num))
        min_line_y = int(centroid_y+(length/2)*sin(pi*i/line_num))
        max_line_x = int(centroid_x+(length/2)*cos(pi*i/line_num))
        max_line_y = int(centroid_y-(length/2)*sin(pi*i/line_num))
        line_arr.append([min_line_x, min_line_y, max_line_x, max_line_y])
        cv.line(rgb_img_copy, (min_line_x, min_line_y), (max_line_x, max_line_y), (0, 0, 255), 2)

    # cv.imshow("grasp_img", rgb_img_copy)
    return line_arr


# 抓取位姿生成器
def grasp_candidate_generator(depth_img, rgb_img, bound_data):
    # thresh = int(bound_data[4])+1
    # depth_img_cut = depth_img[int(bound_data[2]):int(bound_data[3]), int(bound_data[0]):int(bound_data[1])]
    # # 深度滤波和聚类
    # cluster_arr = None
    # while not cluster_arr:
    #     binary = depth_filter(depth_img_cut, thresh)
    #     cluster_img, cluster_arr = dbscan(binary, R, MIN_NUM, thresh)
    #     thresh = thresh+2
    # # 截取深度图准备进行抓取候选生成，注意不要截出图片范围
    # depth_img_height, depth_img_width = depth_img.shape[0], depth_img.shape[1]
    # length = int(max(bound_data[3]-bound_data[2], bound_data[1]-bound_data[0]))
    # # center_x = int(((np.min(cluster_arr[0][:, 0]) + np.max(cluster_arr[0][:, 0]))/2+depth_img_cut.shape[1]/2)/2 +bound_data[0])
    # # center_y = int(((np.min(cluster_arr[0][:, 1]) + np.max(cluster_arr[0][:, 1]))/2+depth_img_cut.shape[0]/2)/2 +bound_data[2])
    # center_x = int((np.min(cluster_arr[0][:, 0]) + np.max(cluster_arr[0][:, 0]))/2*CUT_RATIO+depth_img_cut.shape[1]/2*(1-CUT_RATIO) +bound_data[0])
    # center_y = int((np.min(cluster_arr[0][:, 1]) + np.max(cluster_arr[0][:, 1]))/2*CUT_RATIO+depth_img_cut.shape[0]/2*(1-CUT_RATIO) +bound_data[2])
    # 没有深度滤波聚类直接以识别框为中心截取
    depth_img_height, depth_img_width = depth_img.shape[0], depth_img.shape[1]
    length = int(max(bound_data[3]-bound_data[2], bound_data[1]-bound_data[0]))
    center_x = int((bound_data[1]+bound_data[0])/2)
    center_y = int((bound_data[3]+bound_data[2])/2)
    flag = False
    t = T
    if center_y - t*length >= 0:
        min_y = int(center_y - t*length)
    else:
        min_y = 0
        flag = True
    if center_y + t*length <= depth_img_height:
        max_y = int(center_y + t*length)
    else:
        max_y = depth_img_height
        flag = True
    if center_x - t*length >= 0:
        min_x = int(center_x - t*length)
    else:
        min_x = 0
        flag = True
    if center_x + t*length <= depth_img_width:
        max_x = int(center_x + t*length)
    else:
        max_x = depth_img_width
        flag = True
    # 如果有点超出图片范围则重新截取
    if flag:
        t = min(float(center_y-min_y)/length, float(max_y-center_y)/length, float(center_x-min_x)/length, float(max_x-center_x)/length)
        min_y = int(center_y - t*length)
        max_y = int(center_y + t*length)
        min_x = int(center_x - t*length)
        max_x = int(center_x + t*length)
    # print(min_x, max_x, min_y, max_y)
    # 截取深度图、彩色图之后进行抓取候选生成
    depth_img_cut_new = depth_img[min_y:max_y, min_x:max_x]
    rgb_img_cut_new = rgb_img[min_y:max_y, min_x:max_x]
    line_arr = grasp_generate(depth_img_cut_new, rgb_img_cut_new, LINE_NUM)

    # cv.imshow("depth_cut", depth_img_cut)
    # cv.imshow("binary", binary)
    # cv.imshow("cluster", cluster_img)
    # cv.imshow("rgb_cut", rgb_img_cut_new)

    return depth_img_cut_new, line_arr, (min_x, min_y)