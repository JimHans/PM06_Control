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


def FindBlueOne(frame):
    frame = reshape_image_scan(frame)
    frame = frame[0]
    # 蓝色hsv
    lowHue = 100
    lowSat = 43
    lowVal = 46
    highHue = 124
    highSat = 255
    highVal = 255

    # 红色hsv
    # lowHue = 0
    # lowSat = 43
    # lowVal = 46
    # highHue = 10
    # highSat = 255
    # highVal = 255

    # Show the original image.
    cv2.imshow('frame', frame)

    # Blur methods available, comment or uncomment to try different blur methods.
    frameBGR = cv2.GaussianBlur(frame, (7, 7), 0)
    # frameBGR = cv2.medianBlur(frameBGR, 7)
    # frameBGR = cv2.bilateralFilter(frameBGR, 15 ,75, 75)
    # Show blurred image.
    cv2.imshow('blurred', frameBGR)
    # HSV (Hue, Saturation, Value).
    # Convert the frame to HSV colour model.
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    # HSV values to define a colour range.
    colorLow = numpy.array([lowHue, lowSat, lowVal])
    colorHigh = numpy.array([highHue, highSat, highVal])
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    # Show the first mask
    cv2.imshow('mask-plain', mask)

    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    # Show morphological transformation mask
    cv2.imshow('mask', mask)
    # Put mask over top of the original image.
    image, contours, hierachy = detecte(mask)
    second = FindSecondOne(contours)
    xblue, yblue = compute_center(contours, second)
    print(xblue, yblue)
    cv2.drawContours(image, contours, second, (0, 0, 255), 3)
    cv2.imshow('contours', image)
    cv2.waitKey(0)



if __name__ == '__main__':
    frame = cv2.imread('cam1.jpg')
    FindBlueOne(frame)
    cv2.destroyAllWindows()
