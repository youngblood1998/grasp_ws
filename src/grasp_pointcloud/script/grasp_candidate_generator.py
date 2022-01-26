#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import random
from math import sin, cos, pi

# 深度滤波
def depth_filter(img_path, thresh):
    img = cv.imread(img_path)

    gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    num_arr = np.bincount(gray.flatten())
    fill = np.where(num_arr == np.max(num_arr))
    # gray[gray == 0] = thresh+1
    gray[gray == 0] = fill[0]
    # cv.imshow("binary", gray)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    depth = gray.copy()
    gray[gray >= thresh] = 255
    gray[gray < thresh] = 0

    ret, binary = cv.threshold(gray, 1, 255, cv.THRESH_BINARY_INV)

    cv.imshow("binary", binary)
    cv.waitKey(0)
    cv.destroyAllWindows()

    return binary, depth

# 新的深度滤波
def depth_filter_new(img_path, thresh):
    img = cv.imread(img_path)

    gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

    depth = gray.copy()
    depth[depth == 0] = 255

    num_arr = np.bincount(gray.flatten())
    fill = np.where(num_arr == np.max(num_arr))
    gray[gray == 0] = fill[0]
    print(fill)

    gray[gray >= thresh] = 255
    gray[gray < thresh] = 0

    ret, binary = cv.threshold(gray, 1, 255, cv.THRESH_BINARY_INV)

    # cv.imshow("binary", binary)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    return binary, depth

# 聚类
def dbscan(binary, r, min_num):
    if np.all(binary == 0):
        print("检测深度增加")
        return [], []
    y_idx, x_idx = np.where(binary==255)
    datas = np.vstack((x_idx, y_idx)).T
    clusters = DBSCAN(eps=r, min_samples=min_num).fit_predict(datas)
    np.set_printoptions(threshold=np.inf)

    cluster_img = np.zeros((binary.shape[0], binary.shape[1], 3), dtype=np.uint8)
    color = []
    cluster_arr = []
    for i in range(0, max(clusters)+1):
        color.append([random.randint(0, 256), random.randint(0, 256), random.randint(0, 256)])
        if len(datas[clusters == i]) > 3000 and len(datas[clusters == i]) < 30000:
            cluster_arr.append(datas[clusters == i])
        print(len(datas[clusters == i]))
    if len(cluster_arr) == 0:
        print("检测深度增加")
        return [], []
    print("可抓取物体的数量:{}".format(len(cluster_arr)))
    for i in range(0, len(clusters)):
        cluster_img[y_idx[i], x_idx[i], :] = color[clusters[i]]

    cv.imshow("cluster_img", cluster_img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    return cluster_img, cluster_arr

#抓取姿态生成
def grasp_generation(cluster_img, cluster_arr, line_num=10):
    rectangles_arr = []
    centroids_arr = []
    lines_arr = []
    for i in range(0, len(cluster_arr)):
        x_min, x_max = np.min(cluster_arr[i][:, 0]), np.max(cluster_arr[i][:, 0])
        y_min, y_max = np.min(cluster_arr[i][:, 1]), np.max(cluster_arr[i][:, 1])
        h = y_max-y_min
        w = x_max-x_min
        t = 3/2
        # t = 3
        length = max(h, w)

        # x_min = int(x_min-w/t) if x_min-w/t>0 else 0
        # x_max = int(x_max+w/t) if x_max+w/t>0 else cluster_img.shape[1]
        # y_min = int(y_min-h/t) if y_min-h/t>0 else 0
        # y_max = int(y_max+h/t) if y_max+h/t>0 else cluster_img.shape[0]
        # rectangles_arr.append([y_min, y_max, x_min, x_max])
        # cv.rectangle(cluster_img, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

        centroid_x, centroid_y = int((x_min + x_max)/2), int((y_min + y_max)/2)

        x_min = int(centroid_x-length*t) if centroid_x-length*t>0 else 0
        x_max = int(centroid_x+length*t) if centroid_x+length*t>0 else cluster_img.shape[1]
        y_min = int(centroid_y-length*t) if centroid_y-length*t>0 else 0
        y_max = int(centroid_y+length*t) if centroid_y+length*t>0 else cluster_img.shape[0]
        rectangles_arr.append([y_min, y_max, x_min, x_max])
        cv.rectangle(cluster_img, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

        centroids_arr.append([centroid_x, centroid_y])
        cv.circle(cluster_img, (centroid_x, centroid_y), 2, (0, 0, 255), 2)

        length = min((x_max-x_min), (y_max-y_min))
        length *= 2*t
        line_arr = []
        for i in range(0, line_num):
            min_line_x = int(centroid_x-(length/2)*cos(pi*i/line_num))
            min_line_y = int(centroid_y+(length/2)*sin(pi*i/line_num))
            max_line_x = int(centroid_x+(length/2)*cos(pi*i/line_num))
            max_line_y = int(centroid_y-(length/2)*sin(pi*i/line_num))
            line_arr.append([min_line_x-x_min, min_line_y-y_min, max_line_x-x_min, max_line_y-y_min])
            cv.line(cluster_img, (min_line_x, min_line_y), (max_line_x, max_line_y), (0, 0, 255), 2)
        lines_arr.append(line_arr)

    cv.imshow("grasp_img", cluster_img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    return rectangles_arr, centroids_arr, lines_arr

#抓取姿态生成
def grasp_generation_new(cluster_img, cluster_arr, line_num=10):
    rectangles_arr = []
    centroids_arr = []
    lines_arr = []
    for i in range(0, len(cluster_arr)):
        x_min, x_max = np.min(cluster_arr[i][:, 0]), np.max(cluster_arr[i][:, 0])
        y_min, y_max = np.min(cluster_arr[i][:, 1]), np.max(cluster_arr[i][:, 1])

        centroid_x, centroid_y = int((x_min + x_max)/2), int((y_min + y_max)/2)
        centroids_arr.append([centroid_x, centroid_y])
        cv.circle(cluster_img, (centroid_x, centroid_y), 2, (0, 0, 255), 2)

        h = y_max-y_min
        w = x_max-x_min
        t = 1.6
        length = max(h, w)
        x_min = int(centroid_x-length*t) if centroid_x-length*t>0 else 0
        x_max = int(centroid_x+length*t) if centroid_x+length*t>0 else cluster_img.shape[1]
        y_min = int(centroid_y-length*t) if centroid_y-length*t>0 else 0
        y_max = int(centroid_y+length*t) if centroid_y+length*t>0 else cluster_img.shape[0]
        rectangles_arr.append([y_min, y_max, x_min, x_max])
        cv.rectangle(cluster_img, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

        length *= 2*t
        line_arr = []
        for i in range(0, line_num):
            min_line_x = int(centroid_x-(length/2)*cos(pi*i/line_num))
            min_line_y = int(centroid_y+(length/2)*sin(pi*i/line_num))
            max_line_x = int(centroid_x+(length/2)*cos(pi*i/line_num))
            max_line_y = int(centroid_y-(length/2)*sin(pi*i/line_num))
            line_arr.append([min_line_x-x_min, min_line_y-y_min, max_line_x-x_min, max_line_y-y_min])
            cv.line(cluster_img, (min_line_x, min_line_y), (max_line_x, max_line_y), (0, 0, 255), 2)
        lines_arr.append(line_arr)

    cv.imshow("grasp_img", cluster_img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    return rectangles_arr, centroids_arr, lines_arr

if __name__ == "__main__":
    path = "../img/testDepth1.png"
    binary, depth = depth_filter(path, 44)
    cluster_img, cluster_arr = dbscan(binary, 10, 5)
    grasp_generation(cluster_img, cluster_arr, 10)