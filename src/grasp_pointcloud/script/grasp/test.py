#!/usr/bin/env python
# -*- coding: utf-8 -*-

#---------------------------------------------------------------------------------

# import cv2 as cv
# import rospy
# from sensor_msgs.msg import Image
# import message_filters
# from cv_bridge import CvBridge, CvBridgeError
# import os

# def collect_img(rgb):
#     # print(1)
#     bridge = CvBridge()
#     try:
#         path = "/home/jay/grasp_ws/src/grasp_pointcloud/img"
#         rgb_img = bridge.imgmsg_to_cv2(rgb, "rgb8")
#         # depth_img = bridge.imgmsg_to_cv2(depth, "type_32fc1")
#         if not os.path.exists(path):
#             os.makedirs(path)
#         cv.imwrite(path + "/test.png", rgb_img)
#         # cv.imshow("depth", depth_img)
#         print(rgb_img)
#     except CvBridgeError as e:
#         print(e)

# if __name__ == "__main__":
#     rospy.init_node("collect_img")
#     rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, collect_img, queue_size=1, buff_size=52428800)
#     # depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size=52428800)
#     rospy.spin()

#--------------------------------------------------------------------------------------------------------------------------

# import numpy as np

# random_state = np.random.RandomState(seed=1)
# X = np.concatenate([random_state.normal(-1, 1.5, 350),
#                     random_state.normal(0, 1, 500),
#                     random_state.normal(3, 0.5, 150)]).reshape(-1, 1)
# print(X)

#---------------------------------------------------------------------------------

# import numpy as np
# import pylab as plt
# from scipy.optimize import curve_fit

# x = np.array(range(16))
# y = np.array([0,1,2,3,4,5,5,6,7,8,5,5,4,3,2,1])

# def gaussian(x,*param):
#     return param[0]*np.exp(-np.power(x - param[2], 2.) / (2 * np.power(param[4], 2.)))+\
#            param[1]*np.exp(-np.power(x - param[3], 2.) / (2 * np.power(param[5], 2.)))

# popt,pcov = curve_fit(gaussian,x,y,p0=[3,4,3,6,1,1])
# print(popt)
# print(pcov)
 
# plt.plot(x,y,'b+:',label='data')
# plt.plot(x,gaussian(x,*popt),'ro:',label='fit')
# plt.legend()
# plt.show()

#----------------------------------------------------------------------------

# import cv2 as cv
# from grasp_candidate_generator import depth_filter, dbscan, grasp_generation, depth_filter_new, grasp_generation_new
# from grasp_pose_evaluator_new import gmm, grasp_show

# path = "../img/depth2-2.png"
# thresh = 200
# # binary, depth = depth_filter(path, thresh)
# binary, depth = depth_filter_new(path, thresh)
# cluster_img, cluster_arr = dbscan(binary, 10, 5)
# while len(cluster_img) == 0:
#     thresh += 2
#     binary, depth = depth_filter_new(path, thresh)
#     cluster_img, cluster_arr = dbscan(binary, 10, 5)
# rect_arr, cent_arr, lines_arr = grasp_generation_new(cluster_img, cluster_arr, 10)
# # rect_arr, cent_arr, lines_arr = grasp_generation(cluster_img, cluster_arr, 10)
# # print(lines_arr)
# rgb_path = "../img/rgb2-2.png"
# index_arr = []
# lr_arr = []
# depth_arr = []
# grippers_arr = []
# for i, rect in enumerate(rect_arr):
#     slice_img = depth[rect[0]:rect[1], rect[2]:rect[3]]
#     select_index, lr_index, select_depth, select_width, select_add_width, gripper_arr = gmm(slice_img, lines_arr[i])
#     index_arr.append(select_index)
#     lr_arr.append(lr_index)
#     depth_arr.append(select_depth)
#     grippers_arr.append(gripper_arr)
# grasp_point = grasp_show(rgb_path, index_arr, lr_arr, cent_arr, rect_arr, depth_arr, 10, grippers_arr)
# print(grasp_point)

#-----------------------------------------------------------------------------------------

# import numpy as np

# arr = [5,5,5,5]
# arr_a = arr[:]
# arr_b = arr[:]
# arr_a[2] = arr_a[2]-1
# arr_b[2] = arr_b[2]-1

# arr_a = np.array(arr_a).reshape(4,1)
# matrix = np.array(  [[4,3,2,1],
#                     [5,4,3,2],
#                     [6,5,4,3],
#                     [7,6,5,4]])
# result = np.dot(matrix, arr_a)
# print(result)

#------------------------------------------------------------------

# import rospy
# from std_msgs.msg import Float32MultiArray

# def callback(array):
#     print(type(array.data))

# rospy.init_node("grasp_manipulate")
# array_sub = rospy.Subscriber("/point/grasp_point", Float32MultiArray, callback, queue_size=1, buff_size=52428800)
# rospy.spin()

#-----------------------------------------------------------------------------------------------------------------

# import numpy as np

# arr = np.array([[0,0,0,1],[1,2,3,4],[2,5,1,0],[3,3,3,3]])
# print(sorted(np.unique(arr))[1])

#--------------------------------------------------------------------------------------

# import cv2 as cv
# import numpy as np
# import imputil

# img = np.zeros((1000, 1000, 1), dtype=np.uint8)
# cv.rectangle(img, (700, 300), (800, 700), 255, thickness=-1)
# M = cv.getRotationMatrix2D((500, 500), 45, 1.0)
# img_rotated = cv.warpAffine(img, M, (1000, 1000))
# cv.imshow("img", img)

# cv.waitKey(0)
# cv.destroyAllWindows()

#----------------------------------------------------------------------------

# from goto import with_goto

# class Goto:
#     def __init__(self):
#         self.call_back()

#     @with_goto
#     def call_back(self):
#         i = 1
#         label .there
#         i += 1
#         print(i)
#         while i<5:
#             goto .there

# Goto()

#-----------------------------------------------------------------------------

print(range(10, -1, -1))