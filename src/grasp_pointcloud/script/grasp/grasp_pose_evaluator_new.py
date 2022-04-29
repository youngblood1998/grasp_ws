#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from scipy.optimize import curve_fit
from scipy import interpolate
import pylab as plt
from math import cos,pi
from math import sin, cos, pi

MATRIX = [[1386.60693359375, 0.0, 941.5908813476562], [0.0, 1386.607177734375, 529.2373046875], [0.0, 0.0, 1.0]]
F_DIVIDE_D = 1386.6
GRIPPER_WIDTH = 6
GRASP_NUM = 10

def gaussian(x, a, mu, sigma):
    return a*np.exp(-np.power(x - mu, 2.) / (2 * np.power(sigma, 2.)))

def gaussian_mix3(x, a1, mu1, sigma1, a2, mu2, sigma2, a3, mu3, sigma3):
    return gaussian(x, a1, mu1, sigma1)+gaussian(x, a2, mu2, sigma2)+gaussian(x, a3, mu3, sigma3)

def gaussian_mix2(x, a1, mu1, sigma1, a2, mu2, sigma2):
    return gaussian(x, a1, mu1, sigma1)+gaussian(x, a2, mu2, sigma2)

#计算像素宽度
def cal_width_pix(depth, length):
    width = length*F_DIVIDE_D/depth
    return width

#计算实际长度
def cal_width_real(depth, width):
    length = width*depth/F_DIVIDE_D
    return length

#插值函数
def interpolation(y_arr, x_arr):

    #首尾不能是要插值的
    if y_arr[0] == 255:
        i = 1
        while y_arr[i] == 255:
            i += 1
        y_arr[0] = y_arr[i]
    if y_arr[-1] == 255:
        i = -2
        while y_arr[i] == 255:
            i -= 1
        y_arr[-1] = y_arr[i]
    # print(len(y_arr), len(x_arr))

    true_arr = y_arr < 255
    false_arr = (y_arr == 255)
    insert_x = x_arr[false_arr]
    # print(len(true_arr), len(false_arr), len(insert_x))
    y_arr = y_arr[true_arr]
    x_arr = x_arr[true_arr]
    # print(len(y_arr), len(x_arr))

    f = interpolate.interp1d(x_arr, y_arr, kind='quadratic')
    for x in insert_x:
        y = f(x)
        y_arr = np.insert(y_arr, x, y)
    return y_arr

#高斯混合模型
def gmm(depth_img, lines_arr):

    evaluate_score = 0
    select_index = -1
    lr_index = []
    select_depth = 0
    select_width = 0

    for i, line in enumerate(lines_arr):
        mask = np.zeros((depth_img.shape[0], depth_img.shape[1]), dtype=np.uint8)
        cv.line(mask, (line[0], line[1]), (line[2], line[3]), (1, 1, 1), 1)
        mask = (mask > 0)
        # print(line)
        line_num = depth_img[mask]
        x = np.arange(len(line_num))
        # print(len(line_num))

        line_num = interpolation(line_num, x)

        #减去最小值
        y = line_num - np.min(line_num)

        x = np.arange(len(y))

        try:
            popt,pcov = curve_fit(gaussian_mix2, x, y, bounds=([0, 0, 0, 0, len(y)/2, 0], [np.inf, len(y)/2, np.inf, np.inf, len(y), np.inf]), maxfev = 50000)
            width = cal_width_pix((np.min(line_num)*2+10), GRIPPER_WIDTH*cos(min((i*pi/GRASP_NUM)%(pi/2), pi/2-((i*pi/GRASP_NUM)%(pi/2)))))
            l_depth = gaussian(popt[1]+width/2, popt[0], popt[1], popt[2])
            r_depth = gaussian(popt[4]-width/2, popt[3], popt[4], popt[5])
            min_depth = min(l_depth, r_depth)   #手指可抓的深度
            # grasp_depth = (min_depth + np.min(line_num))*2

            half_depth = min(gaussian(popt[1], popt[0], popt[1], popt[2]), gaussian(popt[4], popt[3], popt[4], popt[5]))/2  #矮峰的半峰高

            # print(popt[2], popt[5])
            # print(popt[0], popt[3])
            if half_depth > min_depth:
                continue

            grasp_depth = (half_depth + np.min(line_num))*2
            # cut_line = y[int(popt[1]):int(popt[4])]
            # l_index = popt[1]
            # r_index = popt[4]
            # for i in range(0, len(cut_line)-1):
            #     if cut_line[i] > half_depth and cut_line[i+1] <= half_depth:
            #         l_index += i
            #         break
            # for i in range(len(cut_line)-1, 0):
            #     if cut_line[i] > half_depth and cut_line[i-1] <= half_depth:
            #         r_index -= i
            #         break
            
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

            add_width = 10
            add_index = cal_width_pix(grasp_depth, add_width)
            add_l_index = l_index-add_index if l_index-add_index>=0 else 0
            add_r_index = r_index+add_index if r_index+add_index<len(y) else len(y)-1
            #测试提取夹爪手指的框
            angle_index = cos(min((i*pi/10)%(pi/2), pi/2-((i*pi/10)%(pi/2))))
            gripper_width = cal_width_pix(grasp_depth, 6*angle_index)
            gripper_length = cal_width_pix(grasp_depth, 23*angle_index)
            if i == 0:
                cp_1_x, cp_1_y = lines_arr[0][0]+int((lines_arr[0][2]-lines_arr[0][0])*add_l_index/len(y)), lines_arr[0][1]
                cp_2_x, cp_2_y = lines_arr[0][0]+int((lines_arr[0][2]-lines_arr[0][0])*add_r_index/len(y)), lines_arr[0][1]
            else:
                cp_1_x, cp_1_y = lines_arr[0][0]+int((lines_arr[0][2]-lines_arr[0][0])*(len(y)-add_r_index)/len(y)), lines_arr[0][1]
                cp_2_x, cp_2_y = lines_arr[0][0]+int((lines_arr[0][2]-lines_arr[0][0])*(len(y)-add_l_index)/len(y)), lines_arr[0][1]
                # cp_1_x, cp_1_y = line[0]+int((line[2]-line[0])*(len(y)-add_r_index)/len(y)), line[1]+int((line[3]-line[1])*(len(y)-add_r_index)/len(y))
                # cp_2_x, cp_2_y = line[0]+int((line[2]-line[0])*(len(y)-add_l_index)/len(y)), line[1]+int((line[3]-line[1])*(len(y)-add_l_index)/len(y))
            p1_1_x, p1_1_y = cp_1_x-int(gripper_width/2), cp_1_y-int(gripper_length/2)
            p1_2_x, p1_2_y = cp_1_x+int(gripper_width/2), cp_1_y+int(gripper_length/2)
            p2_1_x, p2_1_y = cp_2_x-int(gripper_width/2), cp_2_y-int(gripper_length/2)
            p2_2_x, p2_2_y = cp_2_x+int(gripper_width/2), cp_2_y+int(gripper_length/2)
            left_mask = np.zeros((depth_img.shape[0], depth_img.shape[1]), dtype=np.uint8)
            right_mask = np.zeros((depth_img.shape[0], depth_img.shape[1]), dtype=np.uint8)
            cv.rectangle(left_mask, (p1_1_x, p1_1_y), (p1_2_x, p1_2_y), (255, 255, 255), thickness=-1)
            cv.rectangle(right_mask, (p2_1_x, p2_1_y), (p2_2_x, p2_2_y), (255, 255, 255), thickness=-1)
            M = cv.getRotationMatrix2D((int(depth_img.shape[0]/2), int(depth_img.shape[1]/2)), i*180/10, 1.0)
            left_mask = cv.warpAffine(left_mask, M, (depth_img.shape[0], depth_img.shape[1]))
            right_mask = cv.warpAffine(right_mask, M, (depth_img.shape[0], depth_img.shape[1]))
            cv.imshow("left_mask", left_mask)
            cv.imshow("right_mask", right_mask)

            gripper_arr = [p1_1_x, p1_1_y, p1_2_x, p1_2_y, p2_1_x, p2_1_y, p2_2_x, p2_2_y]

            if min(gaussian(add_l_index, popt[0], popt[1], popt[2]), gaussian(add_r_index, popt[3], popt[4], popt[5]))<half_depth:
                continue

            # print(l_index, r_index)
            grasp_width = cal_width_real(grasp_depth, (r_index-l_index)/cos(min((i*pi/GRASP_NUM)%(pi/2), pi/2-((i*pi/GRASP_NUM)%(pi/2)))))

            print(half_depth, min_depth, grasp_width)

            if half_depth*min(popt[2], popt[5])/grasp_width > evaluate_score:
                evaluate_score = half_depth*min(popt[2], popt[5])/grasp_width
                select_index = i
                lr_index = [float(len(y)-l_index*2)/len(y), float(r_index*2-len(y))/len(y)]
                # lr_index = [l_index, r_index]
                # print(len(y), l_index*2, r_index*2)
                # print(lr_index)
                select_depth = grasp_depth
                select_width = grasp_width
                select_add_width = select_width+2*add_width-2*GRIPPER_WIDTH
                select_gripper = gripper_arr[:]
            # grasp_width = cal_width_real(grasp_depth, (popt[4]-popt[1])/cos(min((i*pi/GRASP_NUM)%(pi/2), pi/2-((i*pi/GRASP_NUM)%(pi/2)))))
            # print(grasp_depth, grasp_width)

            # plt.plot(x,y,'b+:',label='data')
            # plt.plot(x,gaussian(x,popt[0], popt[1], popt[2]),'ro:',label='fit1')
            # plt.plot(x,gaussian(x,popt[3], popt[4], popt[5]),'go:',label='fit2')
            # plt.legend()
            # plt.show()
        except RuntimeError:
            print("拟合失败")
        # break
    return select_index, lr_index, select_depth, select_width, select_add_width, select_gripper

def grasp_show(path, index_arr, lr_arr, cent_arr, rect_arr, depth_arr, line_num, grippers_arr):
    # print(index_arr, lr_arr, cent_arr)
    img = cv.imread(path)
    grasp_point = []
    for i, cent in enumerate(cent_arr):
        length = min((rect_arr[i][1]-rect_arr[i][0]), (rect_arr[i][3]-rect_arr[i][2]))
        if index_arr[i]==0:
            min_line_x = int(cent[0]-(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][0])
            min_line_y = int(cent[1]+(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][0])
            max_line_x = int(cent[0]+(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][1])
            max_line_y = int(cent[1]-(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][1])
            # cv.line(img, (min_line_x, min_line_y), (max_line_x, max_line_y), (0, 255, 0), 2)
            # cent_point = [int((min_line_x+max_line_x)/2), int((min_line_y+max_line_y)/2)]
        else:
            # print(cent[i], lr_arr[i])
            min_line_x = int(cent[0]-(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][1])
            min_line_y = int(cent[1]+(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][1])
            max_line_x = int(cent[0]+(length/2)*cos(pi*index_arr[i]/line_num)*lr_arr[i][0])
            max_line_y = int(cent[1]-(length/2)*sin(pi*index_arr[i]/line_num)*lr_arr[i][0])
        cv.line(img, (min_line_x, min_line_y), (max_line_x, max_line_y), (0, 255, 0), 2)
        cent_point = (int((min_line_x+max_line_x)/2), int((min_line_y+max_line_y)/2))
        cv.circle(img, cent_point, 2, (255, 0, 0), 2)

        p1_1 = (rect_arr[i][2]+grippers_arr[i][0], rect_arr[i][0]+grippers_arr[i][1])
        p1_2 = (rect_arr[i][2]+grippers_arr[i][2], rect_arr[i][0]+grippers_arr[i][3])
        p2_1 = (rect_arr[i][2]+grippers_arr[i][4], rect_arr[i][0]+grippers_arr[i][5])
        p2_2 = (rect_arr[i][2]+grippers_arr[i][6], rect_arr[i][0]+grippers_arr[i][7])
        cv.line(img, p1_1, p1_2, (255, 0, 0), 2)
        cv.line(img, p2_1, p2_2, (255, 0, 0), 2)

        pix_point = [int((min_line_x+max_line_x)/2), int((min_line_y+max_line_y)/2), 1]
        point = depth_arr[i]*np.dot(np.matrix(MATRIX).I, np.array(pix_point).T)
        # print(point)
        grasp_point.append([point[0,0], point[0,1], point[0,2]])
    cv.imshow("grasp_show", img)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return grasp_point

if __name__ == "__main__":
    path = "../img/testDepth1.png"
    img = cv.imread(path)
    gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

    # print(gray.shape)