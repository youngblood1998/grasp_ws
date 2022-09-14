#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from scipy.optimize import curve_fit
from scipy import interpolate
from math import cos,pi
from math import sin, cos, pi

from trans_func import trans_real2img_length, trans_img2real_length, trans_img2real_point


GRIPPER_WIDTH = 15  # 夹爪宽
GRIPPER_HEIGHT = 5  # 夹爪厚
SIPPLEMENT_VALUE = 2**15-1    # 空洞填补值
COLLIDE_PERCENT = 0.1   # 可接受碰撞比例
ADD_DEPTH = 20      # 判断宽度所在的深度
STEP = 5            # 步距
DEPTH = 40          # 最大深度
MIN_DEPTH = 20      # 最小抓取深度


#高斯函数
def gaussian(x, a, mu, sigma):
    return a*np.exp(-np.power(x - mu, 2.) / (2 * np.power(sigma, 2.)))

#二重高斯混合
def gaussian_mix2(x, a1, mu1, sigma1, a2, mu2, sigma2):
    return gaussian(x, a1, mu1, sigma1)+gaussian(x, a2, mu2, sigma2)


#插值函数
def interpolation(y_arr, x_arr):
    #首尾不能是要插值的,如首尾要插值先赋予最近的值
    if y_arr[0] == 0:
        i = 1
        while y_arr[i] == 0:
            i += 1
        y_arr[0] = y_arr[i]
    if y_arr[-1] == 0:
        i = -2
        while y_arr[i] == 0:
            i -= 1
        y_arr[-1] = y_arr[i]
    #分离需要插值和用于插值的位置
    true_arr = y_arr > 0
    false_arr = (y_arr == 0)
    #需要插值的位置
    insert_x = x_arr[false_arr]
    #用于插值的点
    y_arr = y_arr[true_arr]
    x_arr = x_arr[true_arr]
    #二次样条插值
    f = interpolate.interp1d(x_arr, y_arr, kind='quadratic')
    for x in insert_x:
        y = f(x)
        y_arr = np.insert(y_arr, x, y)
    #返回插值后的数据
    return y_arr

#高斯混合模型
def gmm(bound_data, depth_img, depth_img_cut, lines_arr):

    best_line_index = -1    # 选择的线段索引
    best_line = None        # 最好的抓取候选
    peak_index = []         # 峰值的索引
    peak_index_close = []   # 闭合后的两指索引
    max_area = 0            # 最大包围面积
    grasp_width_first = 0   # 闭合前的指内宽度
    grasp_width_second = 0  # 闭合后的指内宽度
    grasp_depth = 0         # 抓取深度
    tilt_angle = 0          # 夹爪倾斜角度

    #对每个抓取线进行评估
    for i, line in enumerate(lines_arr):
        #在深度图上将线段取出
        mask = np.zeros((depth_img_cut.shape[0], depth_img_cut.shape[1]), dtype=np.uint8)
        cv.line(mask, (line[0], line[1]), (line[2], line[3]), (1, 1, 1), 1)
        mask = (mask > 0)
        line_num = depth_img_cut[mask]
        if i == 0:
            line_num = line_num[::-1]
        x = np.arange(len(line_num))
        #插值
        line_num = interpolation(line_num, x)
        #减去最小值
        min_depth = np.min(line_num)
        y = line_num - min_depth
        x = np.arange(len(y))

        try:
            #高斯混合模型拟合
            popt,pcov = curve_fit(gaussian_mix2, x, y, bounds=([0, 0, 0, 0, len(y)/2, 0], [np.inf, len(y)/2, np.inf, np.inf, len(y), np.inf]), maxfev = 50000)

            # 截取深度图判断最深抓取深度
            total_length = len(line_num)
            l_percent = popt[1]/(total_length/2)
            r_percent = (total_length-popt[4])/(total_length/2)
            centroid_x, centroid_y, length = depth_img_cut.shape[1]/2, depth_img_cut.shape[0]/2, depth_img_cut.shape[0]-2
            # 计算旋转之后的峰值所在位置
            min_line_x = int(centroid_x-(length/2)*(1-r_percent))
            min_line_y = int(centroid_y)
            max_line_x = int(centroid_x+(length/2)*(1-l_percent))
            max_line_y = int(centroid_y)
            # 计算夹爪的像素宽厚
            real_gripper_width = trans_real2img_length(min_depth, GRIPPER_WIDTH)
            real_gripper_height = trans_real2img_length(min_depth, GRIPPER_HEIGHT)
            # 生成夹爪掩模
            mask_left = np.zeros((depth_img_cut.shape[0], depth_img_cut.shape[1]), dtype=np.uint8)
            mask_right = np.zeros((depth_img_cut.shape[0], depth_img_cut.shape[1]), dtype=np.uint8)
            rect1_point1 = (int(min_line_x-real_gripper_height/2), int(min_line_y-real_gripper_width/2))
            rect1_point2 = (int(min_line_x+real_gripper_height/2), int(min_line_y+real_gripper_width/2))
            rect2_point1 = (int(max_line_x-real_gripper_height/2), int(max_line_y-real_gripper_width/2))
            rect2_point2 = (int(max_line_x+real_gripper_height/2), int(max_line_y+real_gripper_width/2))
            cv.rectangle(mask_right, rect1_point1, rect1_point2, (255, 255, 255), thickness=-1)
            cv.rectangle(mask_left, rect2_point1, rect2_point2, (255, 255, 255), thickness=-1)
            # cv.imshow("mask_left", mask_left)
            # cv.imshow("mask_right", mask_right)
            # cv.waitKey(0)
            mask_left = (mask_left > 0)
            mask_right = (mask_right > 0)
            # 将深度图旋转
            M = cv.getRotationMatrix2D((centroid_x, centroid_y), -i*(180/len(lines_arr)), 1)
            depth_img_cut_rotate = cv.warpAffine(src=depth_img_cut, M=M, dsize=None, borderValue=0)
            depth_img_cut_rotate[depth_img_cut_rotate == 0] = SIPPLEMENT_VALUE
            # 取样后排序得到可接受碰撞比例的值
            num_left = depth_img_cut_rotate[mask_left]
            # num_left = np.sort(num_left)
            num_right = depth_img_cut_rotate[mask_right]
            # num_right = np.sort(num_right)
            num_total = list(num_left) + list(num_right)
            num_total = np.sort(num_total)
            depth_left = num_total[int(COLLIDE_PERCENT*len(num_total))]
            depth_right = num_total[int(COLLIDE_PERCENT*len(num_total))]
            # 计算夹爪之间的面积
            y_cut = y[int(popt[1]):int(popt[4])]
            depth_left_sub = min(depth_left-min_depth, DEPTH)
            depth_right_sub = min(depth_right-min_depth, DEPTH)
            grasp_line = np.linspace(depth_left_sub, depth_right_sub, len(y_cut))
            sub_value = (grasp_line-y_cut).astype(np.int16)
            positive_value = (sub_value[sub_value>0]).astype(np.int16)
            # 判断是否夹到容器
            to_l = int(len(y_cut)/2)
            to_r = int(len(y_cut)/2)
            to_l_flag = False
            to_r_flag = False
            sub_area_1 = 0
            sub_area_2 = 0
            while to_l >= 0:
                if sub_value[to_l] < 0 and not to_l_flag:
                    to_l_flag = True
                if sub_value[to_l] > 0 and to_l_flag:
                    l_percent = (popt[1]+to_l+STEP+GRIPPER_HEIGHT/2)/(total_length/2)
                    sub_area_1 = np.sum(positive_value[0:int(to_l+STEP+GRIPPER_HEIGHT/2)])
                    # print("截断左")
                    break
                to_l = to_l - STEP
            while to_r < len(sub_value):
                if sub_value[to_r] < 0 and not to_r_flag:
                    to_r_flag = True
                if sub_value[to_r] > 0 and to_r_flag:
                    r_percent = (total_length-popt[4]+(len(sub_value)-to_r)+STEP+GRIPPER_HEIGHT/2)/(total_length/2)
                    sub_area_2 = np.sum(positive_value[int(to_r-STEP-GRIPPER_HEIGHT/2):len(sub_value)])
                    # print("截断右")
                    break
                to_r = to_r + STEP
            if not (to_l_flag and to_r_flag):
                continue
            # print(positive_value)
            # print(positive_value)
            angle_ratio = (max(abs(np.cos(np.pi*i/len(lines_arr))), abs(np.sin(np.pi*i/len(lines_arr)))))
            # print(angle_ratio)
            # print("-"*100)
            area = (np.sum(positive_value)-sub_area_1-sub_area_2)/angle_ratio
            # print(area)
            if area > max_area:
                max_area = area
                best_line_index = i
                best_line = line_num
                peak_index = (l_percent, r_percent)
                grasp_depth = max((depth_left+depth_right)/2, MIN_DEPTH)
                grasp_width = trans_img2real_length(grasp_depth, (1-l_percent/2-r_percent/2)*length)
                # tilt_angle = np.arctan((depth_left-depth_right)/grasp_width)
                grasp_width_first = (grasp_width**2+(depth_left-depth_right)**2)**0.5
                # positive = sub_value>0
                # grasp_width_second = grasp_width_first*(np.where(positive==True)[0][-1]-np.where(positive==True)[0][0])/len(positive)
                hirizon_line = np.linspace(ADD_DEPTH, ADD_DEPTH, len(y_cut))
                sub_value_new = hirizon_line-y_cut
                positive = sub_value_new>0
                l_c_index, r_c_index = np.where(positive==True)[0][0], np.where(positive==True)[0][-1]
                peak_index_close = ((popt[1]+l_c_index)/(total_length/2), (total_length-popt[4]+(len(positive)-r_c_index))/(total_length/2))
                grasp_width_second = grasp_width_first*(r_c_index-l_c_index)/len(positive)
                # print(best_line_index, peak_index, grasp_depth, tilt_angle, grasp_width_first, grasp_width_second)
        except RuntimeError:
            print("拟合失败")
    return best_line_index, best_line, peak_index, peak_index_close, grasp_depth, tilt_angle, grasp_width_first, grasp_width_second


#抓取结果展示
def grasp_show(rgb_img, point1, point2):
    img = rgb_img.copy()
    cv.line(img, point1, point2, (0,255,0), 2)
    cv.line(img, point1, point2, (0,255,0), 2)
    return img


def grasp_pose_evaluator(bound_data, depth_img, rgb_img, depth_img_cut, line_arr, cut_img_min_point):
    best_line_index, best_line, peak_index, peak_index_close, grasp_depth, tilt_angle, grasp_width_first, grasp_width_second = gmm(bound_data, depth_img, depth_img_cut, line_arr)
    rotate_angle = best_line_index*np.pi/len(line_arr)
    centroid_x, centroid_y, length = depth_img_cut.shape[1]/2, depth_img_cut.shape[0]/2, depth_img_cut.shape[0]-2
    min_line_x = int(centroid_x-(length/2)*cos(pi*best_line_index/len(line_arr))*(1-peak_index_close[1]))
    min_line_y = int(centroid_y+(length/2)*sin(pi*best_line_index/len(line_arr))*(1-peak_index_close[1]))
    max_line_x = int(centroid_x+(length/2)*cos(pi*best_line_index/len(line_arr))*(1-peak_index_close[0]))
    max_line_y = int(centroid_y-(length/2)*sin(pi*best_line_index/len(line_arr))*(1-peak_index_close[0]))
    result_img = grasp_show(rgb_img, (int(min_line_x+cut_img_min_point[0]), int(min_line_y+cut_img_min_point[1])), (int(max_line_x+cut_img_min_point[0]), int(max_line_y+cut_img_min_point[1])))
    img_point = (cut_img_min_point[0]+(min_line_x+max_line_x)/2, cut_img_min_point[1]+(min_line_y+max_line_y)/2)
    grasp_point = trans_img2real_point(img_point[0], img_point[1], grasp_depth)
    print(grasp_point, rotate_angle, tilt_angle, grasp_width_first-GRIPPER_HEIGHT, grasp_width_second)
    return grasp_point, rotate_angle, tilt_angle, grasp_width_first-GRIPPER_HEIGHT, grasp_width_second, result_img