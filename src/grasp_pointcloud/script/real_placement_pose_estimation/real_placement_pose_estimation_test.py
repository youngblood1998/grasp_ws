#!/usr/bin/env python
# -*- coding: utf-8 -*-
import open3d as o3d
import configparser
import json
import rospy
from sensor_msgs.msg import PointCloud2
from pointcloud_format_func import rospc_to_o3dpc
from placement_pose_estimation_func import pose_estimation


SOURCE_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_placement_pose_estimation/pcd/target_full_box_transform.pcd"
CONFIG_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_placement_pose_estimation/config/pose_config.ini"
RESULT_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_placement_pose_estimation/config/pose_result.ini"

# 节点初始化
rospy.init_node("pose_estimation_test")
rospy.loginfo("放置位姿估计开始!")
# 等待点云数据
msg = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
# 点云格式转换
pcd = rospc_to_o3dpc(msg)
# 读取配准点云
source = o3d.io.read_point_cloud(SOURCE_PATH)
# 位姿估计
frame_value_array = pose_estimation(source=source, target=pcd, config_path=CONFIG_PATH)
# # 将三维列表转换为字符串并写入参数服务器
# list_string = json.dumps(frame_value_array)
# rospy.set_param("/frame_value_array", list_string)
# print(list_string)
# # 从参数服务器读取字符串并将其转换为三维列表
# list_string_new = rospy.get_param("/frame_value_array")
# print("------------------------------------------------------")
# list_3d = json.loads(list_string_new)
# print(list_3d)

# 将三维列表转换为字符串并pose_result.ini中
config_1 = configparser.ConfigParser()
list_string = json.dumps(frame_value_array)
config_1['Result'] = {
    'result': list_string
}
with open(RESULT_PATH, 'w') as configfile:
    config_1.write(configfile)

# 从pose_result.ini中读取字符串并将其转换为三维列表
config_2 = configparser.ConfigParser()
config_2.read(RESULT_PATH)
list_string_new = config_2.get('Result', 'result')
list_3d = json.loads(list_string_new)
print(list_3d)