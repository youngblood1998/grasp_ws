#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import numpy as np

def collect_img(rgb, depth):
    bridge = CvBridge()
    try:
        path = "/home/jay/grasp_ws/src/grasp_pointcloud/img"
        rgb_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
        depth_img = bridge.imgmsg_to_cv2(depth, "64FC1")
        print(np.max(depth_img))
        depth_img = np.round(depth_img/2)
        print(np.max(depth_img))
        if not os.path.exists(path):
            os.makedirs(path)
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        cv.imwrite(path + "/rgb" + time_str + ".png", rgb_img)
        cv.imwrite(path + "/depth" + time_str + ".png", depth_img)
    except CvBridgeError as e:
        print(e)
    rospy.signal_shutdown("collect image finish")


if __name__ == "__main__":
    rospy.init_node("collect_img")
    rgb_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=52428800)
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
    sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 1, 0.005, allow_headerless=True)
    sync.registerCallback(collect_img)
    rospy.spin()
