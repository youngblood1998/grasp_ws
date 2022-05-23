#!/usr/bin/env python2
# -*- coding: utf-8 -*-


# for i in range(10, -1, -1):
#     print(i)

#-------------------------------------

# import numpy as np

# arr = np.array([])
# print(np.mean(arr))

#------------------------------------------------------------------
# def sort_func(a, b):
#     if a[-1]>b[-1]:
#         return -1
#     elif a[-1]<b[-1]:
#         return 1
#     else:
#         return 0

# strawberry_arr = [[0,0,0,0,2], [0,0,0,0,5], [0,0,0,0,1]]
# strawberry_arr.sort(sort_func)
# print(strawberry_arr)

#---------------------------------------------------------------------
# print(False or True)
# a = []
# b = []
# a.extend(b)
# print(len(a))
#----------------------------------------------------------
# a = [5, 6, 1, 3]
# a.sort()
# print(a)
#-----------------------------------------------
# import colorsys
# import random
 
# def get_n_hls_colors(num):
#     hls_colors = []
#     i = 0
#     step = 360.0 / num
#     while i < 360:
#         h = i
#         s = 90 + random.random() * 10
#         l = 50 + random.random() * 10
#         _hlsc = [h / 360.0, l / 100.0, s / 100.0]
#         hls_colors.append(_hlsc)
#         i += step
 
#     return hls_colors

# def ncolors(num):
#     rgb_colors = []
#     if num < 1:
#         return rgb_colors
#     hls_colors = get_n_hls_colors(num)
#     for hlsc in hls_colors:
#         _r, _g, _b = colorsys.hls_to_rgb(hlsc[0], hlsc[1], hlsc[2])
#         r, g, b = [int(x * 255.0) for x in (_r, _g, _b)]
#         rgb_colors.append([r, g, b])
 
#     return rgb_colors

# for i in range(10):
#     print(ncolors(10))
# ---------------------------------------------------
# arr = [0,1,2,3,4,5,6,7,8,9]
# for i in arr:
#     print(i)
#     del arr[0]

#--------------------------------------
# a = float('inf')
# print(a)
# b = 32.5
# print(b>a)
#-------------------------------------
# import numpy as np
# from collections import Counter

# a = np.array([1,2,3,1,2,3,1,2,1,3,3,3,3,2,2,2,2,5,5,5,5,5,5,5])

# count = Counter(a)
# for key,value in count.items():
#     print(value)
#------------------------------------------
# import numpy as np

# a = np.array([[1,2,3],[4,5,6]])
# print(a[:,0])
#----------------------------------------------
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from trans_func import trans_img2real_point
# from sklearn.cluster import DBSCAN
# from collections import Counter
# import time

# plt.ion()

# data1 = np.arange(24).reshape((8, 3))

# x1 = data1[:, 0]  # [ 0  3  6  9 12 15 18 21]
# y1 = data1[:, 1]  # [ 1  4  7 10 13 16 19 22]
# z1 = data1[:, 2]  # [ 2  5  8 11 14 17 20 23]

# data2 = np.random.randint(0, 23, (6, 3))
# x2 = data2[:, 0]
# y2 = data2[:, 1]
# z2 = data2[:, 2]

# fig = plt.figure(1)
# ax = Axes3D(fig)
# ax.scatter(x1, y1, z1, c='r')
# ax.scatter(x2, y2, z2, c='g')

# ax.legend(loc='best')

# ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
# ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
# ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})

# plt.pause(0.01)
# time.sleep(2)

# data1 = np.arange(24).reshape((8, 3))

# x1 = data1[:, 0]  # [ 0  3  6  9 12 15 18 21]
# y1 = data1[:, 1]  # [ 1  4  7 10 13 16 19 22]
# z1 = data1[:, 2]  # [ 2  5  8 11 14 17 20 23]

# data2 = np.random.randint(0, 23, (6, 3))
# x2 = data2[:, 0]
# y2 = data2[:, 1]
# z2 = data2[:, 2]

# fig = plt.figure(1)
# ax = Axes3D(fig)
# ax.scatter(x1, y1, z1, c='r')
# ax.scatter(x2, y2, z2, c='g')

# ax.legend(loc='best')

# ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
# ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
# ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
# plt.pause(0.01)
# time.sleep(2)
#------------------------------------------------------
# import matplotlib.pyplot as plt
# import numpy as np

# def change(arr):
#     x = []
#     y = []
#     for i in range(len(arr)):
#         x.append(arr[i][0])
#         y.append(arr[i][1])
#     x = np.array(x)
#     y = np.array(y)
#     return x, y


# plt.ion()  # 开启交互模式
# plt.subplots()
# plt.xlim(-1, 7)
# plt.ylim(-1, 7)

# arr = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0]]
# arr = np.array(arr)
# x, y = change(arr)
# ax = plt.scatter(x, y)
# for j in range(100):
#     plt.clf()     # 清空画布
#     plt.xlim(-1, 7)      # 因为清空了画布，所以要重新设置坐标轴的范围
#     plt.ylim(-1, 7)
#     for i in range(6):
#         arr[i][0] = np.random.randint(0,6,1)
#         arr[i][1] = np.random.randint(0,6,1)
#     x, y = change(arr)
#     plt.scatter(x, y)
#     plt.pause(0.2)
# plt.ioff()
# plt.show()
#-----------------------------------------

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# data = np.arange(24).reshape((8, 3))

# x = data[:, 0]  # [ 0  3  6  9 12 15 18 21]
# y = data[:, 1]  # [ 1  4  7 10 13 16 19 22]
# z = data[:, 2]  # [ 2  5  8 11 14 17 20 23]

# fig = plt.figure()
# ax = Axes3D(fig)
# ax.scatter(x, y, z, c='r')

# ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
# ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
# ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
# plt.show()
#------------------------------------------------------------
# import pcl

# cloud = pcl.load("/home/jay/grasp_ws/src/grasp_pointcloud/pcd/single1.pcd")
# sor = cloud.make_voxel_grid_filter()
# sor.set_leaf_size(0.1, 0.1, 0.1)
# cloud_filtered = sor.filter()

# print(cloud_filtered.to_array())
#--------------------------------------------------------
import numpy as np

a = np.array([1,2,1])
print(a)
a = a.astype(np.float32)
print(a)