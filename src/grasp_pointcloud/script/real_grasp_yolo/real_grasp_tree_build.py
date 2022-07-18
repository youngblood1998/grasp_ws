#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from trans_func import trans_img2real_point, trans_img2real_length
from random_color import ncolors
import numpy as np
import copy
import random
import cv2 as cv


HALF_LENGTH = 20    # 深度图取样的一半宽度
MIN_LENGTH = 10     # 锚框最小尺寸
MAX_LENGTH = 50     # 锚框最大尺寸
OVERLAP_RATIO = 0.08 # 重叠占比
DEPTH_RATIO = 0.5   # 深度比值
ADD_LENGTH = 10     # 点云直通滤波前后左右各增加的长度
LENGTH_WIGTH_RATIO = 1.4    # 长宽比阈值


# 排序规则
def sort_func(a, b):
    if a[-1] > b[-1]:
        return 1
    elif a[-1] < b[-1]:
        return -1
    else:
        return 0


# 判断是否遮挡
def is_occluded(new_node, node):
    # 没有交叉返回False表示不重叠
    if new_node.data[1]<node.data[0] or new_node.data[0]>node.data[1] or new_node.data[3]<node.data[2] or new_node.data[2]>node.data[3]:
        return False
    else:
        # 计算两个锚框面积
        area1 = (new_node.data[1]-new_node.data[0])*(new_node.data[3]-new_node.data[2])
        area2 = (node.data[1]-node.data[0])*(node.data[3]-node.data[2])
        # 计算交叉部分面积
        x_arr = [new_node.data[0], new_node.data[1], node.data[0], node.data[1]]
        y_arr = [new_node.data[2], new_node.data[1], node.data[0], node.data[1]]
        x_arr.sort()
        y_arr.sort()
        area_cross = (x_arr[2]-x_arr[1])*(y_arr[2]-y_arr[1])
        overlap_ratio = float(area_cross)/min(area1, area2)
        # 判断重叠部分面积是否超过阈值
        if overlap_ratio >= OVERLAP_RATIO:
            return True
        else:
            return False


# 宽度优先遍历
def bfs(new_node, node_arr):
    son_node_arr = []
    ret1 = False # 用来判断是否插入
    # 遍历本层节点
    for node in node_arr:
        # 判断是否重叠且深度在一半以下
        diameter = trans_img2real_length(node.data[4], min(node.data[1]-node.data[0], node.data[3]-node.data[2]))
        depth_sub = new_node.data[4] - node.data[4]
        if is_occluded(new_node, node) and depth_sub > diameter*DEPTH_RATIO:
            node.set_son_node(new_node)
            ret1 = True
        # 准备下一层节点
        for son_node in node.son_ndoe_list:
            if son_node.id == new_node.id:
                continue
            son_node_arr.append(son_node)
    if len(son_node_arr) > 0:
        ret2 = bfs(new_node, son_node_arr)
    else:
        ret2 = False
    return ret1 or ret2


# 定义树的节点结构
class Node:
    def __init__(self, id, data):
        self.id = id    # id用于区分节点
        self.data = data    # 数据
        self.son_ndoe_list = [] # 子节点数组

    def set_son_node(self, node):
        # 先判断节点是否已存在子节点列表，不存在才插入
        exist = False
        for son_node in self.son_ndoe_list:
            if son_node.id == node.id:
                exist = True
                break
        if not exist:
            self.son_ndoe_list.append(node)


# 通过树结构返回最合适的一个草莓锚框数据
def tree_built(depth_img, color_img, bound):
    # print(bound)
    strawberry_arr = []
    # 计算锚框深度并排序
    for b in bound.bounding_boxes:
        # 锚框中心点
        cent_x = int((b.xmin+b.xmax)/2.0)
        cent_y = int((b.ymin+b.ymax)/2.0)
        # 平均深度计算
        # print(cent_y-HALF_LENGTH, cent_y+HALF_LENGTH, cent_x-HALF_LENGTH, cent_x+HALF_LENGTH)
        depth_cut = depth_img[cent_y-HALF_LENGTH:cent_y+HALF_LENGTH, cent_x-HALF_LENGTH:cent_x+HALF_LENGTH]
        depth_cut = depth_cut.flatten()
        depth_cut = depth_cut[depth_cut!=0]
        mean_depth = np.mean(depth_cut)
        # print("深度："+str(mean_depth))
        if len(depth_cut) == 0:
            continue
        # 锚框的长宽计算剔除异常值
        img_x_length = abs(b.xmax-b.xmin)
        img_y_length = abs(b.ymax-b.ymin)
        real_x_length = trans_img2real_length(mean_depth, img_x_length)
        real_y_length = trans_img2real_length(mean_depth, img_y_length)
        # print(real_x_length, real_y_length)
        if (real_x_length < MIN_LENGTH or real_x_length > MAX_LENGTH) or (real_y_length < MIN_LENGTH or real_y_length > MAX_LENGTH):
            continue
        if (real_x_length/real_y_length) > LENGTH_WIGTH_RATIO or (real_y_length/real_x_length) > LENGTH_WIGTH_RATIO:
            continue
        strawberry_arr.append([min(b.xmin, b.xmax), max(b.xmin, b.xmax), min(b.ymin, b.ymax), max(b.ymin, b.ymax), mean_depth])
    # 根据深度值从小往大排序
    strawberry_arr.sort(sort_func)

    # 构建树结构
    node_arr = []
    id = 0
    for strawberry in strawberry_arr:
        new_node = Node(id, strawberry)
        id += 1
        # 第一个节点直接插入
        if len(node_arr) == 0:
            node_arr.append(new_node)
            continue
        # 后续新节点根据是否被遮挡决定是否充当子节点
        ret = bfs(new_node, node_arr)
        # 如果该节点不是某一个节点的子节点，则放第一层
        if not ret:
            node_arr.append(new_node)

    # 在图上画出来
    # print("-"*100)
    # 生成十种颜色
    colors = ncolors(10)
    i = 0
    test_arr = copy.deepcopy(node_arr)
    # 使用宽度遍历画出树结构，每个节点在中心画圆点并用相同的颜色与其子节点相连
    while len(test_arr) != 0:
        color = colors[i]
        i+=1
        new_test_arr = []
        for test in test_arr:
            # print(test.id)
            x_center = int((test.data[0]+test.data[1])/2)
            y_center = int((test.data[2]+test.data[3])/2)
            cv.circle(color_img, (x_center, y_center), 8, color, thickness=5)
            for son in test.son_ndoe_list:
                # print(son.id)
                x = int((son.data[0]+son.data[1])/2)
                y = int((son.data[2]+son.data[3])/2)
                cv.line(color_img, (x_center, y_center), (x, y), color, thickness=3)
                new_test_arr.append(son)
        test_arr = copy.deepcopy(new_test_arr)
        # print("-"*20)
    
    # 在第一层中找到子节点数最多的
    max_son_num = -1
    min_depth = float('inf')
    best_node = None
    for node in node_arr:
        if len(node.son_ndoe_list) > max_son_num:
            max_son_num = len(node.son_ndoe_list)
            min_depth = node.data[4]
            best_node = node
        elif len(node.son_ndoe_list) == max_son_num:
            if node.data[4] < min_depth:
                max_son_num = len(node.son_ndoe_list)
                min_depth = node.data[4]
                best_node = node
    # 计算出点云图的直通滤波范围
    if best_node:
        point_1 = trans_img2real_point(best_node.data[0], best_node.data[2], best_node.data[4])
        point_2 = trans_img2real_point(best_node.data[1], best_node.data[3], best_node.data[4])
        point_bound = [point_1[0]-ADD_LENGTH, point_1[1]-ADD_LENGTH, point_2[0]+ADD_LENGTH, point_2[1]+ADD_LENGTH]

        # 返回画出树结构的图片和锚框数据
        return color_img, point_bound
    else:
        return None, None
