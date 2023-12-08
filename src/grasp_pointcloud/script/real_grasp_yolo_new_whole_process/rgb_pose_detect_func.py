#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

HSV_LOW_RED = np.array([0, 140, 90])
HSV_UP_RED = np.array([180, 255, 120])
HSV_LOW_GREEN = np.array([25, 75, 0])
HSV_UP_GREEN = np.array([55, 230, 70])

def compute_rgb_pose():
    print(2)
    # rospy.init_node("rgb_pose")
    # 订阅图片用于修正姿态
    rgb_img = rospy.wait_for_message("/camera/color/image_raw", Image)
    bridge = CvBridge()
    try:
        # cvbridge转换imgmsg为cv类型
        rgb_img = bridge.imgmsg_to_cv2(rgb_img, "bgr8")
    except CvBridgeError as e:
        print("CvBridge转换出错！！！")
    print(3)
    # 获取原始图像的宽度和高度
    height, width = rgb_img.shape[:2]
    # 计算截取的起始位置和结束位置
    start_x = int(width / 3)
    end_x = int(2 * width / 3)
    start_y = int(height / 4)
    end_y = int(3 * height / 4)
    # 截取图像
    rgb_img = rgb_img[start_y:end_y, start_x:end_x]

    imgHSV = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)   

    # 定义卷积核
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # 获得指定颜色范围内的掩码
    mask_red = cv2.inRange(imgHSV, HSV_LOW_RED, HSV_UP_RED)
    # 计算值为1的像素数量
    count = cv2.countNonZero(mask_red)
    if count < 50:
        print(count)
        return True, 360

    # 开运算闭运算
    opened_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    closed_red = cv2.morphologyEx(opened_red, cv2.MORPH_CLOSE, kernel)
    # 计算掩码的矩
    moments_red = cv2.moments(closed_red)
    # 计算掩码的质心坐标
    center_x_red = moments_red['m10'] / moments_red['m00']
    center_y_red = moments_red['m01'] / moments_red['m00']

    # 获得指定颜色范围内的掩码
    mask_green = cv2.inRange(imgHSV, HSV_LOW_GREEN, HSV_UP_GREEN)
    # 计算值为1的像素数量
    count = cv2.countNonZero(mask_red)
    if count < 10:
        print(count)
        return True, 360
    # 开运算闭运算
    opened_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    closed_green = cv2.morphologyEx(opened_green, cv2.MORPH_CLOSE, kernel)
    # 计算掩码的矩
    moments_green = cv2.moments(closed_green)
    # 计算掩码的质心坐标
    center_x_green = moments_green['m10'] / moments_green['m00']
    center_y_green = moments_green['m01'] / moments_green['m00']

    flag_reverse = center_y_green < center_y_red
    angle_grasp = np.arctan((center_x_green-center_x_red) / (center_y_red-center_y_green)) * 180 / np.pi

    return flag_reverse, angle_grasp

# if __name__ == "__main__":
#     print(1)
#     a, b = compute_rgb_pose()
#     print(a, b)