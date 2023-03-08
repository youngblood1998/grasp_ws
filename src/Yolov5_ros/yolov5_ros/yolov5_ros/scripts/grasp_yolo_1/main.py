#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2 as cv
import time

from grasp_detect_yolo import grasp_yolo_detector
from grasp_tree_build import grasp_tree_builder
from grasp_candidate_generate import grasp_candidate_generator
from grasp_pose_evaluate import grasp_pose_evaluator


# 输入图片
num = 16
rgb_img = cv.imread("img/rgb_"+str(num)+".png")
depth_img = cv.imread("img/depth_"+str(num)+".png")
# rgb_img = cv.imread("img/00380.jpg")

# 展示初始图片
cv.namedWindow("rgb", cv.WINDOW_NORMAL)
cv.imshow("rgb", rgb_img)
cv.namedWindow("depth", cv.WINDOW_NORMAL)
cv.imshow("depth", depth_img)

# 进行转换
depth_img = cv.cvtColor(depth_img, cv.COLOR_RGB2GRAY)
depth_img = np.array(depth_img, dtype = np.int16)
depth_img = depth_img*2

# yolo检测获得所有对象的锚框数据
boxs = grasp_yolo_detector(rgb_img)
start = time.time()
# 树建立并获取抓取对象的xmin, xmax, ymin, ymax, mean_depth
bound_data, remove_line_point = grasp_tree_builder(depth_img, rgb_img, boxs)

rgb_cut = rgb_img[int(bound_data[2]):int(bound_data[3]), int(bound_data[0]):int(bound_data[1])]

cv.namedWindow("rgb_cut_first", cv.WINDOW_NORMAL)
cv.imshow("rgb_cut_first", rgb_cut)
cv.imwrite("result/rgb_rgb_cut_first.png", rgb_cut)

# 抓取候选生成
depth_img_cut, line_arr, cut_img_min_point = grasp_candidate_generator(depth_img, rgb_img, bound_data, remove_line_point)

# 评估抓取候选选择最优
grasp_pose_evaluator(bound_data, depth_img, rgb_img, depth_img_cut, line_arr, cut_img_min_point)
end = time.time()
print(end-start)
# 销毁所有图片
cv.waitKey(0)
cv.destroyAllWindows()