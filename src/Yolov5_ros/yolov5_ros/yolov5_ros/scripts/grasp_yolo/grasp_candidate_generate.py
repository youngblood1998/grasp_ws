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
AREA = 200  # 聚类的类面积阈值


# 深度滤波
def depth_filter(depth_img, thresh):
    # 拷贝并对空洞进行处理
    depth = depth_img.copy()
    depth[depth == 0] = (2**16)-1
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
    
    for i in range(num-1, -1, -1):
        x_min, x_max = np.min(cluster_arr[i][:, 0]), np.max(cluster_arr[i][:, 0])
        y_min, y_max = np.min(cluster_arr[i][:, 1]), np.max(cluster_arr[i][:, 1])
        img_height, img_width = binary.shape[0], binary.shape[1]
        cent_x = (x_min+x_max)/2
        cent_y = (y_min+y_max)/2
        if trans_img2real_length(thresh, x_max-x_min)*trans_img2real_length(thresh, y_max-y_min) < AREA or distance((cent_x, cent_y), (img_width/2, img_height/2)) > min(img_width/2, img_height/2):
            del cluster_arr[i]
    
    if len(cluster_arr) == 0:
        return None, None

    for i in range(0, len(clusters)):
        cluster_img[y_idx[i], x_idx[i], :] = color[clusters[i]]

    return cluster_img, cluster_arr


# 抓取位姿生成器
def grasp_candidate_generator(depth_img, bound_data):
    thresh = int(bound_data[4])
    # print(bound_data[2], bound_data[3], bound_data[0], bound_data[1])
    depth_img_cut = depth_img[int(bound_data[2]):int(bound_data[3]), int(bound_data[0]):int(bound_data[1])]

    cluster_arr = None
    while not cluster_arr:
        binary = depth_filter(depth_img_cut, thresh)
        cluster_img, cluster_arr = dbscan(binary, R, MIN_NUM, thresh)
        thresh = thresh+2

    cv.imshow("depth_cut", depth_img_cut)
    cv.imshow("binary", binary)
    cv.imshow("cluster", cluster_img)