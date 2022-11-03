#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from tf.listener import xyzw_to_mat44
from math import sin, cos
from geometry_msgs.msg import (
    Quaternion,
    Point
)
import pyquaternion


# 根据相机更换
# MATRIX = [[1407.843017578125, 0.0, 973.7594604492188], [0.0, 1407.84326171875, 525.6353759765625], [0.0, 0.0, 1.0]] # 305
MATRIX = [[1386.60693359375, 0.0, 941.5908813476562], [0.0, 1386.607177734375, 529.2373046875], [0.0, 0.0, 1.0]]    # 300
# F_DIVIDE_D = 1407.843017578125  #305
F_DIVIDE_D = 1386.60693359375   #300

# 像素点到实际点
def trans_img2real_point(img_x, img_y, depth):
    img_point = [int(img_x), int(img_y), 1]
    point = depth*np.dot(np.matrix(MATRIX).I, np.array(img_point).T)
    return [point[0,0], point[0,1], point[0,2]]

# 像素长度到实际长度
def trans_img2real_length(depth, length_img):
    length_real = length_img*depth/F_DIVIDE_D
    return length_real

# 实际长度到像素长度
def trans_real2img_length(depth, length_real):
    length_img = length_real*F_DIVIDE_D/depth
    return length_img

# 计算两点距离
def distance(point1, point2):
    return abs(((point2[0]-point1[0])**2+(point2[1]-point1[1])**2)**0.5)

#四元数->变换矩阵
def matrix_from_quaternion(q, p):
    mat44 = xyzw_to_mat44(q)
    mat44[0][3]=p.x
    mat44[1][3]=p.y
    mat44[2][3]=p.z
    return mat44

#欧拉角->旋转矩阵
def euler_to_matrix(theta):
    R_x = np.array([[1, 0,              0,              0],
                    [0, cos(theta[0]),  -sin(theta[0]), 0],
                    [0, sin(theta[0]),  cos(theta[0]),  0],
                    [0, 0,              0,              1]
                    ])
    R_y = np.array([[cos(theta[1]), 0,  sin(theta[1]),  0],
                    [0,             1,  0,              0],
                    [-sin(theta[1]),0,  cos(theta[1]),  0],
                    [0,             0,  0,              1]
                    ])
    R_z = np.array([[cos(theta[2]), -sin(theta[2]), 0,  0],
                    [sin(theta[2]), cos(theta[2]),  0,  0],
                    [0,             0,              1,  0],
                    [0,             0,              0,  1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

# 变换矩阵->四元数
def matrix_to_quaternion(matrix):
    rotate = np.array(matrix[:3, :3])
    q = pyquaternion.Quaternion(matrix = rotate)
    return q

#平移矩阵
def tran_to_matrix(p):
    matrix = np.array(
        [
            [1,0,0,p[0]],
            [0,1,0,p[1]],
            [0,0,1,p[2]],
            [0,0,0,1]
        ]
    )
    return matrix

# 四元数格式转换
def rot_to_ori(rot):
    ori = Quaternion()
    ori.x = rot[0]
    ori.y = rot[1]
    ori.z = rot[2]
    ori.w = rot[3]
    return ori

# 点格式转换
def tran_to_point(trans):
    point = Point()
    point.x = trans[0]
    point.y = trans[1]
    point.z = trans[2]
    return point

# 抓取宽度转换为输入数值
def real_width_to_num(width):
    num = int(250-3.125*width) if int(250-3.125*width)>=0 else 0
    return num

# 夹爪输入数值转增加的长度
def num_to_real_length(num):
    # return 0.04*num
    return -0.0003*num*num + 0.2191*num + 0.8258