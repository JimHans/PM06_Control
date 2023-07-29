#-*-coding:utf-8-*-
# Function: Identify Treasure
#? 宝藏识别模块
#TODO Version 0.2.20230715
#! 依赖项目：OpenCV | numpy
import cv2
import numpy

def reshape_image_scan(image):
    '''归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主'''
    width, height = image.shape[1], image.shape[0]
    min_len = width
    scale = width * 1.0 / 600
    new_width = 600

    new_height = int(height / scale)
    if new_height > 600:
        new_height = 600
        scale = height * 1.0 / 600
        new_width = int(width / scale)
    out = cv2.resize(image, (new_width, new_height))
    return out, new_width, new_height


def compute_center(contours, i):
    '''计算轮廓中心点'''
    M = cv2.moments(contours[i])  # 计算第一条轮廓的各阶矩,字典形式
    cx = int(M['m10'] / M['m00'])  # 计算轮廓中心点
    cy = int(M['m01'] / M['m00'])  # 计算轮廓中心点
    return cx, cy


def detecte(img):
    '''提取所有轮廓'''
    image = cv2.merge([img, img, img])
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)  # 二值化
    contours, hierachy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)# 提取轮廓
    return image, contours, hierachy


def FindMaxOne(AllContours):
    max = 0
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[max]) and i != 0:
            # print("当前最大面积", cv2.contourArea(AllContours[i]))
            max = i
    return max


def FindSecondOne(AllContours):
    max = FindMaxOne(AllContours)
    second = 1
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[second]) and i != max and i != 0:

            second = i
    return second


def FindColorOne(frame,Color):  # (图片，颜色（0蓝，1黄，2绿）)

    frame = reshape_image_scan(frame)
    frame = frame[0]

    colorLow_Blue = numpy.array([100, 43, 46])
    colorHigh_Blue = numpy.array([124, 255, 255])

    colorLow_Yellow = numpy.array([26, 43, 46])
    colorHigh_Yellow = numpy.array([34, 255, 255])

    colorLow_Green = numpy.array([35, 43, 46])
    colorHigh_Green = numpy.array([77, 255, 255])

    if Color == 0:
        colorLow = colorLow_Blue
        colorHigh = colorHigh_Blue
    elif Color == 1:
        colorLow = colorLow_Yellow
        colorHigh = colorHigh_Yellow
    elif Color == 2:
        colorLow = colorLow_Green
        colorHigh = colorHigh_Green

    # Show the original image.
    # cv2.imshow('frame', frame)

    # Blur methods available, comment or uncomment to try different blur methods.
    frameBGR = cv2.GaussianBlur(frame, (7, 7), 0)
    # frameBGR = cv2.medianBlur(frameBGR, 7)
    # frameBGR = cv2.bilateralFilter(frameBGR, 15 ,75, 75)
    # Show blurred image.
    # cv2.imshow('blurred', frameBGR)
    # HSV (Hue, Saturation, Value).
    # Convert the frame to HSV colour model.
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)


    # HSV values to define a colour range.
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    # Show the first mask
    # cv2.imshow('mask-plain', mask)
    # cv2.waitKey(0)
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    # Show morphological transformation mask
    image, contours, hierachy = detecte(mask)
    if Color == 0:
        # cv2.imshow('maskblue', mask)
        print("contours_blue", len(contours))

    elif Color == 1:
        # cv2.imshow('maskyellow', mask)
        print("contours_yellow", len(contours))

    elif Color == 2:
        # cv2.imshow('maskgreen', mask)
        print("contours_green", len(contours))
    # Put mask over top of the original image.


    # 如果轮廓数量小于2，返回原图, 不返回轮廓
    if len(contours) < 2:
        decision = 0
        return frame, None, decision
    else:
        decision = 1
        return image, contours, decision


def FindRedOne(frame,contours):
    copy_contours = contours
    frame = reshape_image_scan(frame)
    frame = frame[0]
    # 蓝色hsv
    # lowHue = 100
    # lowSat = 43
    # lowVal = 46
    # highHue = 124
    # highSat = 255
    # highVal = 255

    # 红色hsv
    lowHue_0 = 0
    lowHue_1 = 156
    lowSat = 100
    lowVal = 100

    highHue_0 = 10
    highHue_1 = 180
    highSat = 255
    highVal = 255

    # Show the original image.
    # cv2.imshow('frame', frame)

    # Blur methods available, comment or uncomment to try different blur methods.
    frameBGR = cv2.GaussianBlur(frame, (15, 15), 10)
    # frameBGR = cv2.medianBlur(frameBGR, 7)
    # frameBGR = cv2.bilateralFilter(frameBGR, 15 ,75, 75)
    # Show blurred image.
    # cv2.imshow('blurred', frameBGR)
    # HSV (Hue, Saturation, Value).
    # Convert the frame to HSV colour model.
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    # HSV values to define a colour range.
    colorLow_0 = numpy.array([lowHue_0, lowSat, lowVal])
    colorHigh_0 = numpy.array([highHue_0, highSat, highVal])
    mask_0 = cv2.inRange(hsv, colorLow_0, colorHigh_0)
    colorLow_1 = numpy.array([lowHue_1, lowSat, lowVal])
    colorHigh_1 = numpy.array([highHue_1, highSat, highVal])
    mask_1 = cv2.inRange(hsv, colorLow_1, colorHigh_1)
    mask = mask_0 + mask_1
    # Show the first mask
    # cv2.imshow('mask-plain', mask)

    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    # Show morphological transformation mask
    # cv2.imshow('maskred', mask)
    # Put mask over top of the original image.
    image, contours, hierachy = detecte(mask)
    print("contours_red", len(contours))
    #如果轮廓数量小于2，返回原图
    if len(contours) < 2:
        decision = 0
        return frame, copy_contours, decision
    else:
        decision = 1
        return image, contours, decision


def reshape_image_scan(image):
    '''归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主'''
    width, height = image.shape[1], image.shape[0]
    min_len = width
    scale = width * 1.0 / 600
    new_width = 600

    new_height = int(height / scale)
    if new_height > 600:
        new_height = 600
        scale = height * 1.0 / 600
        new_width = int(width / scale)
    out = cv2.resize(image, (new_width, new_height))
    return out, new_width, new_height


#定义形状检测函数
def ShapeDetection(copy_img,contours, Treasure):

    # 找到第二大的轮廓
    max = FindMaxOne(contours)
    second = FindSecondOne(contours)
    print("max:",max,"second:",second)
    # 获取第二大轮廓的坐标和长宽
    x, y, w, h = cv2.boundingRect(contours[second])
    cv2.rectangle(copy_img, (x, y), (x+w, y+h), (0, 0, 255), 2)  # 绘制边界框
    cv2.putText(copy_img, Treasure, (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 0), 1)  # 绘制文字

