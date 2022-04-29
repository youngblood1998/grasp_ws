#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from scipy.optimize import curve_fit
from scipy import interpolate
import pylab as plt
from math import sin, cos, pi, log, exp


MATRIX = [[1386.60693359375, 0.0, 941.5908813476562], [0.0, 1386.607177734375, 529.2373046875], [0.0, 0.0, 1.0]]
F_DIVIDE_D = 1386.6

#高斯函数
def gaussian(x, a, mu, sigma):
    return a*np.exp(-np.power(x-mu, 2.)/(2*np.power(sigma, 2.)))

#二重高斯混合
def gaussian_mix2(x, a1, mu1, sigma1, a2, mu2, sigma2):
    return gaussian(x, a1, mu1, sigma1)+gaussian(x, a2, mu2, sigma2)

#实际长度计算像素长度
def cal_width_pix(depth, length):
    width = length*F_DIVIDE_D/depth
    return width

#像素长度计算实际长度
def cal_width_real(depth, width):
    length = width*depth/F_DIVIDE_D
    return length

#插值函数
def interpolation(y_arr, x_arr, max_val):
    #首尾不能是要插值的,如首尾要插值先赋予最近的值
    if y_arr[0] == max_val:
        i = 1
        while y_arr[i] == max_val:
            i += 1
        y_arr[0] = y_arr[i]
    if y_arr[-1] == max_val:
        i = -2
        while y_arr[i] == max_val:
            i -= 1
        y_arr[-1] = y_arr[i]
    #分离需要插值和用于插值的位置
    true_arr = y_arr < max_val
    false_arr = (y_arr == max_val)
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

#高斯混合模型,depth_img深度图,lines_arr线段数组,gripper_height夹爪厚度,grasp_num抓取数,max_val为之前的空洞填补值,add_width抓取余量
def gmm(depth_img, lines_arr, gripper_height, grasp_num, max_val, add_width = 10):
    evaluate_score = 0  #评估分数
    select_index = -1   #选择的线段索引
    lr_index = []       #线段两端百分比
    select_depth = 0    #抓取深度
    select_width = 0    #抓取宽度
    select_add_width = 0#余量抓取宽度
    #对每个抓取线进行评估
    for i, line in enumerate(lines_arr):
        #在深度图上将线段取出
        mask = np.zeros((depth_img.shape[0], depth_img.shape[1]), dtype=np.uint8)
        cv.line(mask, (line[0], line[1]), (line[2], line[3]), (1, 1, 1), 1)
        mask = (mask > 0)
        line_num = depth_img[mask]
        x = np.arange(len(line_num))
        #插值
        line_num = interpolation(line_num, x, max_val)
        #减去最小值
        y = line_num - np.min(line_num)
        x = np.arange(len(y))

        try:
            #高斯混合模型拟合
            popt,pcov = curve_fit(gaussian_mix2, x, y, bounds=([0, 0, 0, 0, len(y)/2, 0], [np.inf, len(y)/2, np.inf, np.inf, len(y), np.inf]), maxfev = 50000)
            #计算夹爪可达最大深度
            angle_index = cos(min((i*pi/grasp_num)%(pi/2), pi/2-((i*pi/grasp_num)%(pi/2))))
            width = cal_width_pix((np.min(line_num)+10), gripper_height*angle_index)
            l_depth = gaussian(popt[1]+width/2, popt[0], popt[1], popt[2])
            r_depth = gaussian(popt[4]-width/2, popt[3], popt[4], popt[5])
            #手指可抓的深度
            min_depth = min(l_depth, r_depth)
            #矮峰的半峰高
            half_depth = min(gaussian(popt[1], popt[0], popt[1], popt[2]), gaussian(popt[4], popt[3], popt[4], popt[5]))/2
            # #不能抓取的情况则跳过
            if half_depth > min_depth:
                continue
            #计算抓取深度
            grasp_depth = half_depth + np.min(line_num)
            #计算半峰高时左右两边是总长的百分比
            l_index = int(len(y)/2)
            r_index = int(len(y)/2)
            for j in range(int(len(y)/2), 0, -1):
                if y[j] < half_depth and y[j-1] > half_depth:
                    l_index = j
                    break
            for k in range(int(len(y)/2), len(y)-1):
                if y[k] < half_depth and y[k+1] > half_depth:
                    r_index = k
                    break
            #计算增加抓取余量后的抓取并判断深度是否合适
            add_index = cal_width_pix(grasp_depth, add_width*angle_index)
            add_l_index = l_index-add_index if l_index-add_index>=0 else 0
            add_r_index = r_index+add_index if r_index+add_index<len(y) else len(y)-1
            if min(gaussian(add_l_index, popt[0], popt[1], popt[2]), gaussian(add_r_index, popt[3], popt[4], popt[5]))<half_depth:
                continue
            #计算抓取宽度
            grasp_width = cal_width_real(grasp_depth, (r_index-l_index)/angle_index)
            #深度不够的情况跳过
            # print(grasp_width, min_depth)
            # if grasp_width/2 > min_depth:
            #     continue
            #评估每个抓取，选择最优（深度、两侧宽度、抓取宽度、对称度）
            if min_depth*(min(popt[2], popt[5])/angle_index)*min(popt[2]*popt[3]/popt[5]/popt[0], popt[5]*popt[0]/popt[2]/popt[3]) > evaluate_score:
                # evaluate_score = half_depth*min(popt[2], popt[5])
                evaluate_score = min_depth*(min(popt[2], popt[5])/angle_index)*min(popt[2]*popt[3]/popt[5]/popt[0], popt[5]*popt[0]/popt[2]/popt[3])
                select_index = i
                lr_index = [float(len(y)-l_index*2)/len(y), float(r_index*2-len(y))/len(y)]
                # select_depth = grasp_depth
                select_depth = grasp_width + np.min(line_num)
                select_width = grasp_width
                select_add_width = select_width+2*add_width-2*gripper_height
        except RuntimeError:
            print("拟合失败")
    #返回抓取的索引、抓取两边的百分比、抓取深度、抓取宽度、增加抓取余量的抓取宽度
    return select_index, lr_index, select_depth, select_width, select_add_width

#抓取结果展示,img为彩色图,index_arr为抓取的索引,lr_arr为抓取两边百分比,cent_arr为中心点,rect_arr为矩形,depth_arr为深度,line_num为总抓取数
def grasp_show(img, index_arr, lr_arr, cent_arr, rect_arr, depth_arr, line_num):
    print(index_arr, lr_arr, cent_arr, rect_arr, depth_arr)
    grasp_point = []
    for i, cent in enumerate(cent_arr):
        length = min((rect_arr[i][1]-rect_arr[i][0]), (rect_arr[i][3]-rect_arr[i][2]))
        #判断正反向并将示意图的抓取宽度变短
        if index_arr[i]==0:
            min_line_x = int(cent[0]-(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][0])
            min_line_y = int(cent[1]+(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][0])
            max_line_x = int(cent[0]+(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][1])
            max_line_y = int(cent[1]-(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][1])
        else:
            min_line_x = int(cent[0]-(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][1])
            min_line_y = int(cent[1]+(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][1])
            max_line_x = int(cent[0]+(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][0])
            max_line_y = int(cent[1]-(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][0])
        cv.line(img, (min_line_x, min_line_y), (max_line_x, max_line_y), (0, 255, 0), 2)
        cent_point = (int((min_line_x+max_line_x)/2), int((min_line_y+max_line_y)/2))
        cv.circle(img, cent_point, 2, (255, 0, 0), 2)
        #计算实际抓取点的位置
        pix_point = [int((min_line_x+max_line_x)/2), int((min_line_y+max_line_y)/2), 1]
        point = depth_arr[i]*np.dot(np.matrix(MATRIX).I, np.array(pix_point).T)
        grasp_point.append([point[0,0], point[0,1], point[0,2]])
    #返回抓取点和示意图
    return grasp_point, img