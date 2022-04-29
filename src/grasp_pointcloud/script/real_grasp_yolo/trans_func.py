#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np


MATRIX_305 = [[1407.843017578125, 0.0, 973.7594604492188], [0.0, 1407.84326171875, 525.6353759765625], [0.0, 0.0, 1.0]]
MATRIX_300 = [[1386.60693359375, 0.0, 941.5908813476562], [0.0, 1386.607177734375, 529.2373046875], [0.0, 0.0, 1.0]]
F_DIVIDE_D_305 = 1407.843017578125
F_DIVIDE_D_300 = 1386.60693359375

# 像素点到实际点
def trans_img2real_point(img_x, img_y, depth):
    img_point = [int(img_x), int(img_y), 1]
    point = depth*np.dot(np.matrix(MATRIX_305).I, np.array(img_point).T)
    return [point[0,0], point[0,1], point[0,2]]

# 像素长度到实际长度
def trans_img2real_length(depth, length_img):
    length_real = length_img*depth/F_DIVIDE_D_305
    return length_real

# 实际长度到像素长度
def trans_real2img_length(depth, length_real):
    length_img = length_real*F_DIVIDE_D_305/depth
    return length_img