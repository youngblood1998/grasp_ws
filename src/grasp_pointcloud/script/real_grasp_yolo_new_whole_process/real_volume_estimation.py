#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from volume_estimate_func import compute_strawberry_volume
from pointcloud_format_func import rospc_to_o3dpc
from sensor_msgs.msg import PointCloud2
from grasp_pointcloud.msg import VolumeParams


class VolumeEstimation:

    def __init__(self):
        self.volume_params_pub = rospy.Publisher("real_detect/volume_params", VolumeParams, queue_size=1)
        self.point_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback, queue_size=1, buff_size=52428800)

    def callback(self, point_cloud):
        #如果参数grasp_step不存在或者不等于2说明机器人在移动,暂停检测
        if rospy.has_param("/grasp_step") and int(rospy.get_param("/grasp_step"))!=2:
            return 0
        pcd = rospc_to_o3dpc(point_cloud)
        position_grasp, angle_grasp, flag_reverse, volume, width = compute_strawberry_volume(pcd)
        volume_params = VolumeParams()
        volume_params.reverse = flag_reverse
        volume_params.x = position_grasp[0]
        volume_params.y = position_grasp[1]
        volume_params.z = position_grasp[2]
        volume_params.rotate_angle = angle_grasp
        volume_params.volume = volume
        volume_params.width = width
        self.volume_params_pub.publish(volume_params)


if __name__ == "__main__":
    rospy.init_node("volume_estimation")
    rospy.loginfo("重量检测开始")
    VolumeEstimation()
    rospy.spin()