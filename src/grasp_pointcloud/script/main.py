#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
from grasp_candidate_generator import depth_filter, dbscan, grasp_generation
from grasp_pose_evaluator import ddi, gmm

path = "../img/depth2-2.png"
thresh = 200
binary, depth = depth_filter(path, thresh)
cluster_img, cluster_arr = dbscan(binary, 10, 5)
while len(cluster_img) == 0:
    thresh += 2
    binary, depth = depth_filter(path, thresh)
    cluster_img, cluster_arr = dbscan(binary, 10, 5)
rect_arr, cent_arr, lines_arr = grasp_generation(cluster_img, cluster_arr, 10)
# print(lines_arr)
for i, rect in enumerate(rect_arr):
    slice_img = depth[rect[0]:rect[1], rect[2]:rect[3]]
    ddi_img = ddi(slice_img, 7)
    gmm(ddi_img, lines_arr[i])