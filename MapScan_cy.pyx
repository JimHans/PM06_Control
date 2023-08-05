#-*-coding:utf-8-*-
# Function: MapScan algorithm
#? 地图扫描算法实现模组
#TODO Version 1.0.20230805
#! 依赖项目：numpy | copy | OpenCV | numba
#! 被引用：main.py
import numpy as np
import copy, cv2


def reshape_image_scan(image): #* 归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主
    width, height = image.shape[1], image.shape[0]
    scale = width * 1.0 / 600
    new_width = 600
    new_height = int(height / scale)
    if new_height > 600:
        new_height = 600
        scale = height * 1.0 / 600
        new_width = int(width / scale)
    out = cv2.resize(image, (new_width, new_height))
    return out, new_width, new_height


def detect(image): #* 提取所有轮廓
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)      # 二值化
    contours, hierachy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # 提取轮廓
    print("轮廓提取完成")
    return image, contours, hierachy #* 返回轮廓


def detect_blue(img): #* 提取蓝色轮廓
    image = cv2.merge([img, img, img])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)  # 二值化
    contours, hierachy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)# 提取轮廓
    return image, contours, hierachy


def compute_1(contours, i, j): #* 最外面的轮廓和子轮廓的比例
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    if area2 == 0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio ):
        return True
    return False


def compute_2(contours, i, j): #* 子轮廓和子子轮廓的比例
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    if area2 == 0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio ):
        return True
    return False


def compute_center(contours, i):#* 计算轮廓中心点
    M = cv2.moments(contours[i])  # 计算第一条轮廓的各阶矩,字典形式
    cx = int(M['m10'] / M['m00'])  # 计算轮廓中心点
    cy = int(M['m01'] / M['m00'])  # 计算轮廓中心点
    return cx, cy


def detect_contours(vec):#* 判断这个轮廓和它的子轮廓以及子子轮廓的中心的间距是否足够小
    distance_1 = np.sqrt((vec[0] - vec[2]) ** 2 + (vec[1] - vec[3]) ** 2)
    distance_2 = np.sqrt((vec[0] - vec[4]) ** 2 + (vec[1] - vec[5]) ** 2)
    distance_3 = np.sqrt((vec[2] - vec[4]) ** 2 + (vec[3] - vec[5]) ** 2)
    if sum((distance_1, distance_2, distance_3)) / 3 < 3:
        return True
    return False


def find(image, contours, hierachy, root=0):#* 找到符合要求的轮廓
    rec = []
    for i in range(len(hierachy)):
        child = hierachy[i][2]
        child_child = hierachy[child][2]
        if child != -1 and hierachy[child][2] != -1:
            if compute_1(contours, i, child) and compute_2(contours, child, child_child):
                cx1, cy1 = compute_center(contours, i)
                cx2, cy2 = compute_center(contours, child)
                cx3, cy3 = compute_center(contours, child_child)
                if detect_contours([cx1, cy1, cx2, cy2, cx3, cy3]):
                    rec.append([cx1, cy1, i, child, child_child])
    #? 计算得到所有在比例上符合要求的轮廓中心点
    xblue, yblue = FindBlueOne(image)
    #? 以距离xblue，yblue这个点最近的点为第0个点，将四个轮廓中心点按顺时针排序
    if len(rec) != 0:
        rec = sorted(rec, key=lambda x: x[0])
        slope = [0,0,0,0]
        for i in range(len(rec)-1): # 计算斜率
            slope[i+1] = (rec[i+1][1] - rec[0][1]) / (rec[i+1][0] - rec[0][0])
        h = [0,0,0,0]
        max = -100000
        for i in range(len(slope)-1):
            if slope[i+1] > max:
                max = slope[i+1]
                h[1] = i+1
        
        max = -100000
        for i in range(len(slope)-1): # 找到第二大的斜率
            if slope[i+1] > max and i+1 != h[1]:
                max = slope[i+1]
                h[2] = i+1
        
        max = -100000
        for i in range(3): # 找到第三大的斜率
            if slope[i+1] >max and i+1 != h[1] and i+1 != h[2] :
                max = slope[i+1]
                h[3] = i+1
        print("h1,h2,h3斜率计算:", h[1], h[2], h[3])
        recs = [[0, 0, 0] for i in range(4)]

        for i in range(4):
            recs[i][0] = rec[h[i]][0]
            recs[i][1] = rec[h[i]][1]
        # print("recs:", recs)
        rec = recs

        min = 100000; k = 0 
        for i in range(len(rec)):  # 找到距离最近的点

            if np.sqrt((rec[i][0] - xblue) ** 2 + (rec[i][1] - yblue) ** 2) < min:
                min = np.sqrt((rec[i][0] - xblue) ** 2 + (rec[i][1] - yblue) ** 2)
                k = i # ;print("k", k)

        recx = [[0, 0] for i in range(4)]
        for j in range(4):
            # print("j,j+k,j+k-4", j, j+k, j+k-4)
            if j+k < 4:
                recx[j][0] = rec[j+k][0]
                recx[j][1] = rec[j+k][1]
            else:
                recx[j][0] = rec[j+k-4][0]
                recx[j][1] = rec[j+k-4][1]

    recx = np.array(recx)
    box1 = np.array([[recx[0][0], recx[0][1]], [recx[1][0], recx[1][1]], [recx[3][0], recx[3][1]], [recx[2][0], recx[2][1]]])
    result = copy.deepcopy(image)  # 拷贝原图
    cv2.drawContours(result, [box1], 0, (0, 0, 255), 2)  # 连接定位点中点
    return recx, image


def affine_transformation(image, rec, new_width, new_height):
    '''透视变换'''
    pts1 = np.float32([[rec[1][0], rec[1][1]], [rec[0][0], rec[0][1]], [rec[2][0], rec[2][1]], [rec[3][0], rec[3][1]]])
    pts2 = np.float32([[0, 0], [new_width, 0], [0, new_height], [new_width, new_height]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(image, M, (new_width, new_height))
    return dst


def FindBlueOne(frame): #* 找到最近的蓝色点
    frame = reshape_image_scan(frame)
    frame = frame[0]
    # 蓝色hsv
    lowHue  = 100;lowSat  = 43; lowVal  = 46
    highHue = 124;highSat = 255;highVal = 255

    # 红色hsv
    # lowHue = 0;lowSat = 43;lowVal = 46
    # highHue = 10;highSat = 255;highVal = 255

    # 高斯模糊
    frameBGR = cv2.GaussianBlur(frame, (7, 7), 0)
    # 转换成HSV
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    # 设定蓝色阈值
    colorLow = np.array([lowHue, lowSat, lowVal])
    colorHigh = np.array([highHue, highSat, highVal])
    # 根据阈值构建掩膜
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    # 对原图像和掩膜进行位运算
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    # 检测轮廓
    image, contours, hierachy = detect_blue(mask)
    second = FindSecondOne(contours)
    xblue, yblue = compute_center(contours, second)
    # 画出轮廓
    cv2.drawContours(image, contours, second, (0, 0, 255), 3)
    return xblue, yblue


def FindMaxOne(AllContours): #* 找到最大的轮廓
    max = 0
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[max]) and i != 0:max = i
    return max


def FindSecondOne(AllContours): #* 找到第二大的轮廓
    max = FindMaxOne(AllContours)
    second = 1
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[second]) and i != max and i != 0:second = i
    return second
