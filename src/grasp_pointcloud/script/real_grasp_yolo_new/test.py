#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import tf
from geometry_msgs.msg import (
    Quaternion,
    Point
)
from trans_func import matrix_from_quaternion, rot_to_ori, tran_to_point
import pyquaternion

TRAN = [0.0, -0.0524528072638, 0.0739784679795] #手眼标定的平移
ROT = [-0.0322859285682, -0.00222200140914, -0.0294295826053, 0.999042832512]   #手眼标定的旋转


ori_cam_to_end = rot_to_ori(ROT)
point_cam_to_end = tran_to_point(TRAN)
matrix_cam_to_end = matrix_from_quaternion(ori_cam_to_end, point_cam_to_end)
matrix_cam_to_end_ = np.linalg.inv(matrix_cam_to_end)

rotate = np.array(matrix_cam_to_end[:3, :3])
print(rotate)
q = pyquaternion.Quaternion(matrix = rotate)
print(q.x)
# print(matrix_cam_to_end_, matrix_cam_to_end)
# print(np.dot(matrix_cam_to_end, matrix_cam_to_end_))