#!/usr/bin/env python
# -*- coding: utf-8 -*-
import open3d as o3d
import configparser
import json
import rospy
from sensor_msgs.msg import PointCloud2
from pointcloud_format_func import rospc_to_o3dpc
from placement_pose_estimation_func import pose_estimation


SOURCE_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_grasp_yolo_new_whole_process/pcd/target_full_box_transform.pcd"
CONFIG_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_grasp_yolo_new_whole_process/config/pose_config.ini"
RESULT_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_grasp_yolo_new_whole_process/config/pose_result.ini"


class PlacementPoseEstimation:
    
    def __init__(self):
        print(1)
        num = 1
        config = configparser.ConfigParser()
        config['Result'] = {}
        while input("检测第{}个盒子的槽位，按任意键继续，按回车结束".format(num)):
            # 等待点云数据
            msg = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
            # 点云格式转换
            pcd = rospc_to_o3dpc(msg)
            # 读取配准点云
            source = o3d.io.read_point_cloud(SOURCE_PATH)
            # 位姿估计
            frame_value_array = pose_estimation(source=source, target=pcd, config_path=CONFIG_PATH)
            # 将三维列表转换为字符串并pose_result.ini中
            list_string = json.dumps(frame_value_array)
            config['Result']['result_'+str(num)] = list_string
            num += 1
        with open(RESULT_PATH, 'w') as configfile:
            config.write(configfile)


if __name__ == "__main__":
    # 节点初始化
    rospy.init_node("pose_estimation_test")
    rospy.loginfo("放置位姿估计开始!")
    PlacementPoseEstimation()