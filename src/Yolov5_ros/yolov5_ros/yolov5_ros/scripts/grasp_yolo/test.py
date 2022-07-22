# import cv2
# import numpy

# cvtImage = cv2.imread("/home/jay/grasp_ws/src/Yolov5_ros/yolov5_ros/yolov5_ros/scripts/grasp_yolo/plt_img/pic-0.png")
# rows, cols = cvtImage.shape[:2]
# center = (cols / 2, rows / 2)
# angle = 30
# scale = 1

# M = cv2.getRotationMatrix2D(center, angle, scale)
# cvtImage_rotate = cv2.warpAffine(src=cvtImage, M=M, dsize=None, borderValue=(0, 0, 0))

# cv2.imshow("rotate", cvtImage_rotate)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
#-----------------------------------------
# print(2**16-1)
import numpy as np

# a = np.array(
#     [1.1,2.2,3.3,4.4]
# )
# b = a>2
# print(b)

# # b = np.array(
# #     [4,5,6,7]
# # )
# # print(b-a)
# c = np.where(b==True)
# print(c[0][0])
a = np.linspace(10,10,10)
print(a)