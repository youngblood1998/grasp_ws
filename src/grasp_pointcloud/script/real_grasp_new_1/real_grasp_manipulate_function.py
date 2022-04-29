#!/usr/bin/env python
# -*- coding: utf-8 -*-
from tf.listener import xyzw_to_mat44
import numpy as np
from math import sin, cos

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
