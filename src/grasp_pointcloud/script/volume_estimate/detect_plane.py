#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import ros_numpy as rn
import numpy as np


# ros的PointCloud2点云数据转numpy数组
def pointcloud2_to_array(cloud_msg):
    # Use ros_numpy to convert pointcloud2 to numpy array
    point_cloud = rn.numpify(cloud_msg)
    # Create a new numpy array with the XYZ coordinates and RGB values
    point_array = []
    for i in range(point_cloud.shape[0]):
        point_array.append([point_cloud[i]['x'], point_cloud[i]['y'], point_cloud[i]['z']])
    return point_array

# 通过x，y值计算平面上的z值
def cal_point(plane_parameters, point_x, point_y):
    return -(plane_parameters[0]*point_x + plane_parameters[1]*point_y + plane_parameters[3])/plane_parameters[2]

# 计算平面相对于相机的变换矩阵4×4
def cal_matrix(plane_parameters):
    # 计算两个点的z值
    z_1 = cal_point(plane_parameters, 0, 0)
    z_2 = cal_point(plane_parameters, 1, 0)
    # 获得x轴两点
    point_1 = np.array([0, 0, z_1])
    point_2 = np.array([1, 0, z_2])
    # 获得x轴向量
    x_axis = point_2 - point_1
    norm = np.linalg.norm(x_axis)
    x_axis_norm = x_axis / norm
    z_axis_norm = np.array(plane_parameters[:3])
    y_axis_norm = np.cross(z_axis_norm, x_axis_norm)
    # 返回平面坐标
    return np.transpose(np.vstack((np.insert(x_axis_norm, 3, 0),
                                   np.insert(y_axis_norm, 3, 0),
                                   np.insert(z_axis_norm, 3, 0),
                                   [0, 0, -plane_parameters[3], 1])))


if __name__ == "__main__":
    # 初始化ros节点并等待点云数据
    rospy.init_node("detect_plane")
    rospy.loginfo("检测平面程序开始")
    ros_pc = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
    # 点云格式转换
    points = np.array(pointcloud2_to_array(ros_pc))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    # 直通滤波
    # 定义过滤范围
    x_min, x_max = -0.1, 0.1   # x轴过滤范围
    y_min, y_max = -0.1, 0.1   # y轴过滤范围
    z_min, z_max = 0, 1.0   # z轴过滤范围
    # 进行直通滤波
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
    pcd_filtered = pcd.crop(bbox)

    # 平面分割
    plane_model, inliers = pcd_filtered.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=100)
    inlier_cloud = pcd_filtered.select_down_sample(inliers)     # 平面
    outlier_cloud = pcd_filtered.select_down_sample(inliers, invert=True)       # 平面之外的点云
    plane_parameters = np.asarray(plane_model)
    z_axis = plane_parameters[:3]        # 平面的法向量
    print('平面法向量的值:' + str(plane_parameters))

    # 计算变换矩阵并设置到参数服务器
    matrix = cal_matrix(plane_parameters)
    rospy.set_param('plane_matrix', matrix.tolist())
    print("检测平面完成")