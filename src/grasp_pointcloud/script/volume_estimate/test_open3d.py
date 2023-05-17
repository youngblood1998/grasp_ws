#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import open3d
from sensor_msgs.msg import Image

rospy.sleep(5)
tree_pub = rospy.Publisher("real_detect/tree_image", Image, queue_size=1)
# import open3d as o3d
# import numpy as np
#
# # 加载点云数据
# pcd = o3d.io.read_point_cloud("../../pcd/single3.pcd")
#
# # 定义过滤范围
# x_min, x_max = -0.1, 0.1   # x轴过滤范围
# y_min, y_max = -0.1, 0.1   # y轴过滤范围
# z_min, z_max = 0, 1.0   # z轴过滤范围
#
# # 进行直通滤波
# bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
# pcd_filtered = pcd.crop(bbox)
#
# # 平面分割
# plane_model, inliers = pcd_filtered.segment_plane(distance_threshold=0.003,
#                                          ransac_n=3,
#                                          num_iterations=1000)
# inlier_cloud = pcd_filtered.select_down_sample(inliers)
# outlier_cloud = pcd_filtered.select_down_sample(inliers, invert=True)
#
# # 离群点滤波
# cl, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=10,
#                                                     std_ratio=1.0)
#
# # 计算PCA
# cl.estimate_normals()
# center = cl.get_center()
# points = np.asarray(cl.points)
# points -= center
# cov = np.dot(points.T, points) / points.shape[0]
#
# # Singular value decomposition.
# eigen_values, eigen_vectors = np.linalg.eig(cov)
# idx = eigen_values.argsort()[::-1]
# eigen_values = eigen_values[idx]
# eigen_vectors = eigen_vectors[:, idx]
#
# # 获取主方向向量
# main_direction = eigen_vectors[:, 0]
#
# # 创建一个坐标系，其中x轴对齐于主方向向量
# coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.005,
#                                                                   origin=[0, 0, 0])
#
# # 将坐标系沿着主方向平移到点云的中心位置
# coord_frame.translate(center)
#
# # 将点云和坐标系一起可视化
# o3d.visualization.draw_geometries([cl, coord_frame])

#----------------------------------------------------------
# import open3d as o3d
# import numpy as np
#
# # 加载点云文件
# pcd = o3d.io.read_point_cloud("../../pcd/single2.pcd")
#
# # 将点云转换为NumPy数组，以便进行RGB值的过滤
# points = np.asarray(pcd.points)
# colors = np.asarray(pcd.colors)
#
# # 设置颜色筛选条件
# red_channel_values = colors[:, 0]
# green_channel_values = colors[:, 1]
# color_filter = ((red_channel_values > 0.2) & (red_channel_values < 0.4)) | (green_channel_values > 0) & (green_channel_values < 0.4)
#
# # 使用颜色筛选条件选择点云
# filtered_points = points[color_filter]
# filtered_colors = colors[color_filter]
#
# # 创建新的点云对象并保存
# filtered_pcd = o3d.geometry.PointCloud()
# filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
# color = [1, 0, 1]
# filtered_pcd.colors = o3d.utility.Vector3dVector([color for i in range(len(filtered_pcd.points))])
#
# print(filtered_pcd)
#
# # 计算主方向和包围盒
# bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(filtered_pcd.points)
# obb = filtered_pcd.get_oriented_bounding_box()
# print(np.asarray(bbox.get_box_points()))
# print(np.asarray(obb.get_box_points()))
#
# color = [1, 1, 0]
#
# # 可视化结果
# o3d.visualization.draw_geometries([pcd, filtered_pcd, obb], "origin")
#----------------------------------------------------------------------------
# import open3d as o3d
# import numpy as np
#
# # 假设原始坐标系的z轴为(0,0,1)
# # 平面法向量为(0,0,1)
#
# # 计算旋转矩阵
#
# z_axis = np.array([0, 0, 1])    # 原始坐标系的z轴单位向量
# normal = np.array([0, 1, 1])    # 平面法向量单位向量
#
# axis = np.cross(z_axis, normal)    # 计算旋转轴，即原始坐标系z轴与平面法向量的叉乘
#                                    # 叉乘结果是垂直于这两个向量的向量，也就是旋转轴
# angle = np.arccos(np.dot(z_axis, normal))    # 计算旋转角度，即原始坐标系z轴到平面法向量的夹角
#                                              # 因为两个向量都是单位向量，所以可以用点积求出它们之间的夹角
# print(angle)
# R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis, angle)    # 根据旋转轴和旋转角度生成旋转矩阵
#-------------------------------------------------------------------------------------------------------
# import open3d as o3d
# import numpy as np
#
# # 平面法向量
# n = np.array([0.03627258, -0.00300772,  0.99933741])
#
# # 找到平面法向量在三维坐标系中的投影方向
# z = np.array([0, 0, 1])
# proj_dir = np.cross(n, z)
# proj_dir = proj_dir / np.linalg.norm(proj_dir)
#
# # 找到第二个与平面法向量和其投影方向都垂直的向量
# perp_dir = np.cross(n, proj_dir)
# perp_dir = perp_dir / np.linalg.norm(perp_dir)
#
# # 构造旋转矩阵
# rot_mat = np.array([proj_dir, perp_dir, n])
#
# print(rot_mat)
#---------------------------------------------------
# import numpy as np
#
# # 定义一个平面的法向量和点
# plane_normal = np.array([0, 1, 0])
# plane_point = np.array([0, 0, 0])
#
# # 定义要投影的向量
# vector_to_project = np.array([1, 2, 3])
#
# # 计算向量在平面上的投影向量
# projection_vector = vector_to_project - np.dot(vector_to_project, plane_normal) / np.dot(plane_normal, plane_normal) * plane_normal
#
# # 打印投影向量
# print(projection_vector)
#-------------------------------------
# import numpy as np
#
# transformation_matrix = np.array([[-0.08266272, -0.99657758,  0.,         -0.01350621],
#  [ 0.99591727, -0.08260795, -0.03639669, -0.00574353],
#  [ 0.03627212, -0.00300865,  0.99933742,  0.35444077],
#  [ 0.,          0.,         0.,          1.        ]])
# # 假设 transformation_matrix 为 4x4 的变换矩阵
# T = transformation_matrix[0:3, 0:3]   # 提取旋转矩阵部分
# y_axis = T[:, 1]                      # 提取该坐标系的 Y 轴方向
# y_unit_vector = y_axis / np.linalg.norm(y_axis)   # 转化为单位向量
# print(y_unit_vector)
#-------------------------------------------------------------------
# import numpy as np
# import open3d as o3d
#
# # 定义平面参数
# A = 1
# B = 2
# C = 3
# D = 4
#
# # 计算平面法向量和截距
# n = np.array([A, B, C])
# d = -D / C
#
# # 创建平面模型的顶点和三角形索引列表
# vertices = np.array([
#     [-10, -10, d],
#     [10, -10, d],
#     [10, 10, d],
#     [-10, 10, d]
# ])
#
# triangles = np.array([
#     [0, 1, 2],
#     [0, 2, 3]
# ])
#
# # 创建平面模型
# plane = o3d.geometry.TriangleMesh()
# plane.vertices = o3d.utility.Vector3dVector(vertices)
# plane.triangles = o3d.utility.Vector3iVector(triangles)
#
# # 创建可视化窗口并添加平面模型
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(plane)
#
# # 显示平面模型
# o3d.visualization.draw_geometries([plane])