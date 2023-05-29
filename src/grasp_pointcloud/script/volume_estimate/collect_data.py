#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import open3d as o3d
from sensor_msgs.msg import PointCloud2, Image
import ros_numpy as rn
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import os


# ros的PointCloud2点云数据转numpy数组
def pointcloud2_to_array(cloud_msg):
    # Use ros_numpy to convert pointcloud2 to numpy array
    point_cloud = rn.numpify(cloud_msg)
    # Convert the RGB values to 0~1
    rgb = point_cloud['rgb']
    b = (rgb & 0xff) / 255.0
    g = ((rgb >> 8) & 0xff) / 255.0
    r = ((rgb >> 16) & 0xff) / 255.0
    # Create a new numpy array with the XYZ coordinates and RGB values
    point_array = []
    for i in range(point_cloud.shape[0]):
        point_array.append([point_cloud[i]['x'], point_cloud[i]['y'], point_cloud[i]['z'], r[i], g[i], b[i]])
    return point_array

# 回调函数，保存点云和图片
def callback(name):
    # 相机输入的点云和图像数据
    pointcloud = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
    rgb = rospy.wait_for_message("/camera/color/image_raw", Image)
    depth = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
    # 图像格式转换及保存
    bridge = CvBridge()
    try:
        rgb_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
        depth_img = bridge.imgmsg_to_cv2(depth, "64FC1")
        depth_img = np.round(depth_img / 2)
        path = "/home/jay/grasp_ws/src/grasp_pointcloud/script/volume_estimate/"
        if not os.path.exists(path):
            os.makedirs(path)
        cv.imwrite(path + "rgb_img/" + str(name) + ".png", rgb_img)
        cv.imwrite(path + "depth_img/" + str(name) + ".png", depth_img)
    except CvBridgeError as e:
        print(e)
    # 点云格式转换及保存
    points = pointcloud2_to_array(pointcloud)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(points[:, 3:])
    o3d.io.write_point_cloud(path + "pcd/" + str(name) + ".pcd", pcd)
    print("保存完成: "+str(name))

# 初始化ros节点
rospy.init_node("collect_data")
rospy.loginfo("数据采集程序开始")
sub = rospy.Subscriber("collect/collectable", String, callback, queue_size=1, buff_size=52428800)
rospy.spin()