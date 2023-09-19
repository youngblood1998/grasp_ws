import cv2 as cv
import numpy as np

# def callback(a):
#     thresh = cv.getTrackbarPos("Thresh", "TrackBars")
#     return thresh

img = cv.imread("./rgb_img/box6.png")
img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
img_med = cv.medianBlur(img_gray, 7)

# # 限制对比度的自适应阈值均衡化
# clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(16, 16))
# img_clahe = clahe.apply(img_gray)

# 进行圆检测
circles = cv.HoughCircles(img_med, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=45, param2=20, minRadius=0,
                           maxRadius=20)
# 若检测到圆，则绘制圆
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")
    for (x, y, r) in circles:
        cv.circle(img_med, (x, y), r, (0, 255, 0), 2)  # 绘制圆
        cv.circle(img_med, (x, y), 3, (0, 0, 255), -1)  # 绘制圆心
# cv.imshow("img", img)
# cv.imshow("img_gray", img_gray)
cv.imshow("img_med", img_med)

cv.waitKey(0)
cv.destroyAllWindows()

# cv.namedWindow("TrackBars")
# cv.resizeWindow("TrackBars", 640, 240)
# cv.createTrackbar("Thresh", "TrackBars", 0, 255, callback)
#
# while True:
#     thresh = callback(0)
#     _, img_bin = cv.threshold(img_clahe, thresh, 255, cv.THRESH_BINARY)
#
#     # 进行圆检测
#     circles = cv.HoughCircles(img_bin, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=0,
#                                maxRadius=0)
#     # 若检测到圆，则绘制圆
#     if circles is not None:
#         circles = np.round(circles[0, :]).astype("int")
#         for (x, y, r) in circles:
#             cv.circle(img_bin, (x, y), r, (0, 255, 0), 2)  # 绘制圆
#             cv.circle(img_bin, (x, y), 3, (0, 0, 255), -1)  # 绘制圆心
#
#     # 显示结果图像
#     cv.imshow("Circle Detection", img_bin)
#     cv.waitKey(1)

# cv.imshow("img", img)
# cv.imshow("img_gray", img_gray)
# cv.imshow("img_med", img_med)
# # cv.imshow("img_clahe", img_clahe)
#
# cv.waitKey(0)
# cv.destroyAllWindows()