#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
import sys
from scipy.optimize import curve_fit
import pylab as plt

#深度差异图
def ddi(depth_img, m=3):
    ddi_img = np.zeros((depth_img.shape[0]-m+1, depth_img.shape[1]-m+1), dtype=np.int8)
    for i in range(0, depth_img.shape[0]-m+1):
        for j in range(0, depth_img.shape[1]-m+1):
            cut = depth_img[i:i+m, j:j+m]
            center_val = cut[(m-1)/2, (m-1)/2]
            cut = cut-center_val
            cut[(m-1)/2, (m-1)/2] = -sys.maxint-1
            ddi_img[i, j] = np.max(cut)
    b = int(m/2)
    ddi_img = cv.copyMakeBorder(ddi_img, b, b, b, b, cv.BORDER_REPLICATE)
    # cv.imshow("ddi", ddi_img)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    return ddi_img

def gaussian(x, a, mu, sigma):
    return a*np.exp(-np.power(x - mu, 2.) / (2 * np.power(sigma, 2.)))

def gaussian_mix3(x, a1, mu1, sigma1, a2, mu2, sigma2, a3, mu3, sigma3):
    return gaussian(x, a1, mu1, sigma1)+gaussian(x, a2, mu2, sigma2)+gaussian(x, a3, mu3, sigma3)

def gaussian_mix2(x, a1, mu1, sigma1, a2, mu2, sigma2):
    return gaussian(x, a1, mu1, sigma1)+gaussian(x, a2, mu2, sigma2)

#高斯混合模型
def gmm(ddi_img, lines_arr):
    for line in lines_arr:
        mask = np.zeros((ddi_img.shape[0], ddi_img.shape[1]), dtype=np.uint8)
        cv.line(mask, (line[0], line[1]), (line[2], line[3]), (1, 1, 1), 1)
        mask = (mask > 0)
        # print(line)
        line_num = ddi_img[mask]

        #所有数减去最小的数使大于0
        # min_val = np.min(line_num)
        # print(line_num)
        # line_num -= min_val
        # print(line_num)

        #小于0的取绝对值
        line_num = np.abs(line_num)

        #小于0的取0
        # line_num[line_num < 0] = 0

        x = np.arange(len(line_num))
        # print(x)
        # popt,pcov = curve_fit(gaussian_mix3, x, line_num, p0=[10,len(line_num)/8,2,10,len(line_num)/2,2,10,len(line_num)*7/8,2], maxfev = 50000)
        try:
            # popt,pcov = curve_fit(gaussian_mix3, x, line_num, maxfev = 50000)
            # popt,pcov = curve_fit(gaussian_mix3, x, line_num, bounds=([0, 0, 0, 0, 0, 0, 0, len(line_num)/2, 0], [np.inf, len(line_num)/2, np.inf, np.inf, len(line_num), np.inf, np.inf, len(line_num), np.inf]), maxfev = 50000)
            popt,pcov = curve_fit(gaussian_mix2, x, line_num, bounds=([0, 0, 0, 0, len(line_num)/2, 0], [np.inf, len(line_num)/2, np.inf, np.inf, len(line_num), np.inf]), maxfev = 50000)
            plt.plot(x,line_num,'b+:',label='data')
            plt.plot(x,gaussian(x,popt[0], popt[1], popt[2]),'ro:',label='fit1')
            plt.plot(x,gaussian(x,popt[3], popt[4], popt[5]),'go:',label='fit2')
            # plt.plot(x,gaussian(x,popt[6], popt[7], popt[8]),'yo:',label='fit3')
            plt.legend()
            plt.show()
        except RuntimeError:
            print("拟合失败")
        # break

if __name__ == "__main__":
    path = "../img/testDepth1.png"
    img = cv.imread(path)
    gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

    ddi_img = ddi(np.array(gray), 3)
    print(ddi_img.shape)
    print(gray.shape)
    # cv.imshow("ddi_img", ddi_img)
    # cv.waitKey(0)
    # cv.destroyAllWindows()