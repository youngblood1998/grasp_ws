#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from trans_func import trans_img2real_point, trans_img2real_length
import numpy as np

HALF_LENGTH = 20    # 深度图取样的一半宽度
MIN_LENGTH = 10     # 锚框最小尺寸
MAX_LENGTH = 60     # 锚框最大尺寸

def tree_built(depth_img, bound, overlap_ratio):
    print(bound)
    # 对每一个锚框遍历生成树结构并选取抓取对象
    for b in bound.bounding_boxes:
        # 锚框中心点
        cent_x = int((b.xmin+b.xmax)/2.0)
        cent_y = int((b.ymin+b.ymax)/2.0)
        # 平均深度计算
        print(cent_y-HALF_LENGTH, cent_y+HALF_LENGTH, cent_x-HALF_LENGTH, cent_x+HALF_LENGTH)
        depth_cut = depth_img[cent_y-HALF_LENGTH:cent_y+HALF_LENGTH, cent_x-HALF_LENGTH:cent_x+HALF_LENGTH]
        depth_cut = depth_cut.flatten()
        depth_cut = depth_cut[depth_cut!=0]
        mean_depth = np.mean(depth_cut)
        print("深度："+str(mean_depth))
        if len(depth_cut) == 0:
            continue
        # 锚框的长宽计算剔除异常值
        img_x_length = abs(b.xmax-b.xmin)
        img_y_length = abs(b.ymax-b.ymin)
        real_x_length = trans_img2real_length(mean_depth, img_x_length)
        real_y_length = trans_img2real_length(mean_depth, img_y_length)
        print(real_x_length, real_y_length)
        if (real_x_length < MIN_LENGTH or real_x_length > MAX_LENGTH) or (real_y_length < MIN_LENGTH or real_y_length > MAX_LENGTH):
            continue
    print("-"*50)