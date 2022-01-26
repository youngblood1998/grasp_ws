#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import random
from math import sin, cos, pi


# 深度滤波,depth_img是原始深度图,thresh是滤波深度
def depth_filter_new(depth_img_input, thresh):
    depth_img = depth_img_input.copy()
    #填补空洞后用于返回的深度图depth
    depth = depth_img_input.copy()
    max_val = np.max(depth)+1
    depth[depth == 0] = max_val
    #将空洞填补为频率最高的值
    num_arr = np.bincount(depth_img.flatten())
    num_arr[0] = 0  #消除空洞值最多的影响
    fill = np.where(num_arr == np.max(num_arr))
    depth_img[depth_img == 0] = fill[0]
    #二值化处理
    depth_img[depth_img < thresh] = 0
    depth_img[depth_img >= thresh] = 255
    depth_img = 255-depth_img   #相当于二值化中的cv.THRESH_BINARY_INV
    # ret, binary = cv.threshold(depth_img, 1, 255, cv.THRESH_BINARY_INV)
    #返回二值图,填补后的深度图,以及填补的值
    return depth_img, depth, max_val

# 聚类,binary二值图,r和min_num是基于密度的聚类的参数
def dbscan(binary, r, min_num):
    #滤波深度不够
    if np.all(binary == 0):
        return [], []
    #基于密度的聚类
    y_idx, x_idx = np.where(binary==255)
    datas = np.vstack((x_idx, y_idx)).T
    clusters = DBSCAN(eps=r, min_samples=min_num).fit_predict(datas)
    #构造聚类后的示意图,不同的类用不同的颜色表示
    cluster_img = np.zeros((binary.shape[0], binary.shape[1], 3), dtype=np.uint8)
    color = []
    cluster_arr = []
    for i in range(0, max(clusters)+1):
        color.append([random.randint(0, 256), random.randint(0, 256), random.randint(0, 256)])
        if len(datas[clusters == i]) > 4000 and len(datas[clusters == i]) < 12000:
            cluster_arr.append(datas[clusters == i])
    #滤波深度不够
    if len(cluster_arr) == 0:
        return [], []
    #不同类给不同颜色
    print("可抓取物体的数量:{}".format(len(cluster_arr)))
    for i in range(0, len(clusters)):
        cluster_img[y_idx[i], x_idx[i], :] = color[clusters[i]]
    #返回聚类示意图和聚类结果
    return cluster_img, cluster_arr

#抓取姿态生成,cluster_arr聚类数组,line_num每个类的抓取线段数,t是矩形倍数
def grasp_generation_new(cluster_arr, cluster_img, line_num=10, t=1.5):
    #矩形数组、中心点数组和抓取线数组
    rectangles_arr = []
    centroids_arr = []
    lines_arr = []
    #对每个类计算矩形、中心点和抓取线
    for i in range(0, len(cluster_arr)):
        #算类在两个轴的最大最小值
        x_min, x_max = np.min(cluster_arr[i][:, 0]), np.max(cluster_arr[i][:, 0])
        y_min, y_max = np.min(cluster_arr[i][:, 1]), np.max(cluster_arr[i][:, 1])
        #计算类的中心
        centroid_x, centroid_y = int((x_min + x_max)/2), int((y_min + y_max)/2)
        centroids_arr.append([centroid_x, centroid_y])
        #计算矩形两端点
        h = y_max-y_min
        w = x_max-x_min
        length = max(h, w)
        x_min = int(centroid_x-length*t) if centroid_x-length*t>0 else 0
        x_max = int(centroid_x+length*t) if centroid_x+length*t>0 else cluster_img.shape[1]
        y_min = int(centroid_y-length*t) if centroid_y-length*t>0 else 0
        y_max = int(centroid_y+length*t) if centroid_y+length*t>0 else cluster_img.shape[0]
        rectangles_arr.append([y_min, y_max, x_min, x_max])
        #计算抓取线端点
        length *= 2*t
        line_arr = []
        for i in range(0, line_num):
            min_line_x = int(centroid_x-(length/2)*cos(pi*i/line_num))
            min_line_y = int(centroid_y+(length/2)*sin(pi*i/line_num))
            max_line_x = int(centroid_x+(length/2)*cos(pi*i/line_num))
            max_line_y = int(centroid_y-(length/2)*sin(pi*i/line_num))
            line_arr.append([min_line_x-x_min, min_line_y-y_min, max_line_x-x_min, max_line_y-y_min])
        lines_arr.append(line_arr)
    #返回矩形数组、中心点数组和线段数组
    return rectangles_arr, centroids_arr, lines_arr