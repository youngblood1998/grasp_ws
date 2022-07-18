#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np


def grasp_yolo_detector(image):
    # 初始化yolo模型参数
    model = torch.hub.load("../../yolov5", 'custom', path="../../weights/strawberry_x.pt", source='local')
    model.cuda()
    model.conf = 0.3
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #结果 xmin    ymin    xmax   ymax  confidence  class    name
    results = model(image_rgb)
    boxs = results.pandas().xyxy[0].values
    # 检测结果可视化
    dectshow(image, boxs, image_rgb.shape[0], image_rgb.shape[1])
    return boxs

def dectshow(org_img, boxs, height, width):
        img = org_img.copy()
        img_circle = org_img.copy()

        count = 0
        for i in boxs:
            count += 1
        
        # 依次画锚框和圆
        for box in boxs:
            cv2.rectangle(img, (int(box[0]), int(box[1])),(int(box[2]), int(box[3])), (0, 255, 0), 2)
            cv2.putText(img, box[-1],(int(box[0]), int(box[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(img_circle, (int((box[0]+box[2])/2), int((box[1]+box[3])/2)), int(max(box[2]-box[0], box[3]-box[1])/2), (0, 255, 0), 2)
            cv2.putText(img_circle, box[-1],(int(box[0]), int(box[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('detect_img_rect', img)
        cv2.imshow('detect_img_circle', img_circle)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()