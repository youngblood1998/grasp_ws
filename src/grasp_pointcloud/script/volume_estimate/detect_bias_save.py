import cv2
import numpy as np
import math

def color_segment(path, hsv_min, hsv_max):
    img = cv2.imread(path)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array(hsv_min)
    upper = np.array(hsv_max)
    # 获得指定颜色范围内的掩码
    mask = cv2.inRange(imgHSV, lower, upper)
    # 对原图图像进行按位与的操作，掩码区域保留
    imgResult = cv2.bitwise_and(img, img, mask=mask)

    # 定义卷积核
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # 开运算
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 闭运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

    # cv2.imshow("Mask", mask)
    # cv2.imshow("Result", imgResult)
    #
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return closed

def cal_moments(binary):
    # 寻找非零像素的坐标
    coords = cv2.findNonZero(binary)

    # 计算最小包围矩形
    rect = cv2.minAreaRect(coords)

    # 将矩形坐标转换为整数型
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # 绘制最小包围矩形
    cv2.drawContours(binary, [box], 0, (255, 0, 0), 2)

    # 计算最小包围矩形的长边角度
    edge1 = box[0] - box[1]
    edge2 = box[1] - box[2]
    long_edge = edge1 if np.linalg.norm(edge1) > np.linalg.norm(edge2) else edge2
    angle = np.arctan2(-long_edge[0], long_edge[1]) * 180 / np.pi - 180
    print(angle)

    # 显示绘制结果
    cv2.imshow('Min Area Rect', binary)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def draw_angle(path):
    # 读入图片
    img = cv2.imread(path)

    mask = color_segment(path, [0, 106, 0], [179, 255, 255])

    # 计算图像的矩
    M = cv2.moments(mask)

    # 计算图像的中心点
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    # # 获取图片中心坐标及长宽
    # h, w = img.shape[:2]
    # cx, cy = w // 2, h // 2

    # 计算半径和直线长度
    radius = min(cx, cy) * 0.9
    line_length = 50

    # 初始化角度和步长
    angle = 0
    step = 2

    # 循环画线并标注角度
    while angle < 180:
        # 根据当前角度计算直线起点和终点的坐标
        x1 = int(cx + radius * math.cos(math.radians(angle)))
        y1 = int(cy - radius * math.sin(math.radians(angle)))
        x2 = int(cx + (radius + line_length) * math.cos(math.radians(angle)))
        y2 = int(cy - (radius + line_length) * math.sin(math.radians(angle)))

        # 画直线
        cv2.line(img, (cx, cy), (x1, y1), (255, 255, 255), 1)

        # 标注角度
        if angle <= 90:
            cv2.putText(img, str(angle), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cv2.putText(img, str(angle), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 更新角度
        angle += step

    # 显示图片
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    for i in range(1, 21):
        path = './rgb_img/{}-0.25-0.png'.format(str(i))
        draw_angle(path)
        # binary = color_segment(path, [0, 106, 0], [179, 255, 255])
        # cal_moments(binary)