#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import ros_numpy as rn
import numpy as np


# ros的点云格式转open3d点云格式
def rospc_to_o3dpc(cloud_msg):
    # Use ros_numpy to convert pointcloud2 to numpy array
    point_cloud = rn.numpify(cloud_msg)
    # Create a new numpy array with the XYZ coordinates and RGB values
    point_array = []
    for i in range(point_cloud.shape[0]):
        point_array.append([point_cloud[i]['x'], point_cloud[i]['y'], point_cloud[i]['z']])
    # 点云格式转换
    points = np.array(point_array)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    return pcd
