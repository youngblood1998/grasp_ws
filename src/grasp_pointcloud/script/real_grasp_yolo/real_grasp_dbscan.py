#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trans_func import trans_img2real_point
from sklearn.cluster import DBSCAN
from collections import Counter
import pcl
from sensor_msgs.msg import PointCloud2
import time


R_VAl = 5       # DBSCAN的r值
MIN_NUM = 10    # DBSCAN的min_samples值

def dbscan(depth_img, bound_data):
    start = time.time()
    # 深度图转换为点云
    point_arr = []
    for y in range(bound_data[2], bound_data[3]):
        for x in range(bound_data[0], bound_data[1]):
            real_point = trans_img2real_point(x, y, depth_img[y, x])
            point_arr.append(real_point)
    point_arr = np.array(point_arr).astype(np.float32)
    print(time.time()-start)
    # 转化为pcl点云
    point_cloud = pcl.PointCloud()
    point_cloud.from_array(point_arr)
    print(time.time()-start)
    # 进行统计学离群点去除
    fil = point_cloud.make_statistical_outlier_filter()
    fil.set_mean_k(20)
    fil.set_std_dev_mul_thresh(2.0)
    point_cloud = fil.filter()
    print(time.time()-start)
    # 进行体素降采样
    sor = point_cloud.make_voxel_grid_filter()
    sor.set_leaf_size(3, 3, 3)
    point_cloud = sor.filter()
    point_arr = point_cloud.to_array()
    print(time.time()-start)
    # DBSCAN聚类
    clusters = DBSCAN(eps=R_VAl, min_samples=MIN_NUM).fit_predict(point_arr)
    count = Counter(clusters)
    print(time.time()-start)
    # 将每个类分离并找出数量最多的类
    max_length = 0
    index_max = -1
    cluster_arr = []
    for i in range(0, max(clusters)+1):
        cluster_arr.append(point_arr[clusters == i])
        if len(point_arr[clusters == i]) > max_length:
            max_length = len(point_arr[clusters == i])
            index_max = i
    print(time.time()-start)
    
    # # 将聚类结果表示出来
    # fig = plt.figure(1)
    # ax = Axes3D(fig)
    # # 把数量最多的类用红色表示，其他用绿色
    # j = 0
    # for cluster in cluster_arr:
    #     if j == index_max:
    #         x = cluster[:, 0]
    #         y = cluster[:, 1]
    #         z = cluster[:, 2]
    #         ax.scatter(x, y, z, c="r")
    #     else:
    #         x = cluster[:, 0]
    #         y = cluster[:, 1]
    #         z = cluster[:, 2]
    #         ax.scatter(x, y, z, c="g")
    #     j += 1
    
    # ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    # ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    # ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    # plt.show()