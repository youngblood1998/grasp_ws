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
print(2**16-1)