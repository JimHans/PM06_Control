#-*-coding:utf-8-*-
# Function: Find Ways algorithm
#? 寻路算法实现模组
#TODO Version 0.2.20230723
#! 依赖项目：numpy | OpenCV
#! 被引用：main.py
import cv2
import numpy as np


def reshape_image_find(image): #* 归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主
    width, height = image.shape[1], image.shape[0]
    # min_len = width
    scale = width * 1.0 / 800
    new_width = 800
    new_height = int(height / scale)
    if new_height > 800:
        new_height = 800
        scale = height * 1.0 / 800
        new_width = int(width / scale)
    out = cv2.resize(image, (new_width, new_height))
    return out, new_width, new_height


def ToBinray(image): #* 转二进制图像
    global imgray, binary
    # 1、灰度图
    imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 2、二进制图像
    # ret, binary = cv2.threshold(imgray, 127, 255, 0)
    blur = cv2.GaussianBlur(imgray, (5, 5), 0)
    ret, binary=cv2.threshotd (blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return ret, binary


def FindMaxOne(AllContours): #* 框出最大的轮廓
    max = 0
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[max]) and i != 0:
            # print("当前最大面积", cv2.contourArea(AllContours[i]))
            max = i
    return max


def FindSecondOne(AllContours): #* 框出第二大的轮廓
    max = FindMaxOne(AllContours)
    second = 1
    for i in range(len(AllContours)):
        if cv2.contourArea(AllContours[i]) > cv2.contourArea(AllContours[second]) and i != max and i != 0:
            second = i
    return second


def GetGontours(image): #* 提取轮廓
    # 1、根据二值图找到轮廓
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # 2、画出轮廓
    maxone = FindMaxOne(contours)
    secondone = FindSecondOne(contours)
    dst = cv2.drawContours(image, contours, secondone, (255, 0, 255), 1)
    # print("画框面积为", cv2.contourArea(contours[secondone]))
    cropimg = BoundingRect(image, contours, secondone)
    cropimgclone = cropimg.copy()
    # get length and width of cropimg
    w, h = cropimg.shape[1], cropimg.shape[0]
    print("w,h:", w, h)
    cell = int(h * 1.0 / 20)

    treasure, treasureinmap = findcircles(cropimgclone, cell, cropimg)
    _, board,cell = pic(cropimg)
    for i in range(8):
        cv2.circle(cropimg, (treasure[i][0], treasure[i][1]), 2, (255, 0, 255), 2)
    for i in range(len(treasureinmap)):
        cv2.circle(cropimg, (cell * treasureinmap[i][0] + int(cell * 3), cell * treasureinmap[i][1] + int(cell * 1)), 2, (255, 0, 0), 2)

    return contours, board, cropimg, cell, treasureinmap


def BoundingRect(img, contours, whichone=0): #* 获取边界矩形
    # 1、取外围轮廓
    cnt = contours[whichone]
    # 2、获取正方形坐标长宽
    x, y, w, h = cv2.boundingRect(cnt)
    # 3、画出矩形
    dst = img.copy()
    # 按矩形将图片裁剪
    cropImg = img[y:y + h, x:x + w]
    #将cropImg宽和高设置为900*750
    cropImg = cv2.resize(cropImg, (600, 500))
    return cropImg


def pic(imm):  #* 腐蚀加厚黑边
    ret, binary = ToBinray(imm)
    k = np.ones((25, 25), np.uint8)  # 定义核
    binary = cv2.erode(binary, k)  # 腐蚀
    board, cell = getBoard(binary, imm)
    return binary, board, cell


def findcircles(smarties,cell,cropimg):  #* 找圆
    gray_img = cv2.cvtColor(smarties, cv2.COLOR_BGR2GRAY)
    # 进行中值滤波
    img = cv2.medianBlur(gray_img, 5)
    # 进行霍夫圆变换
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20, circles=None, param1=50, param2=26, minRadius=5,
                                maxRadius=35)
    print("circles",circles)
    # treasure存储圆心坐标
    # 对数据进行四舍五入变为整数
    circles = np.uint16(np.around(circles))
    treasure = []
    treasureinmap = []
    for i in circles[0, :]:
        # 画出来圆心
        cv2.circle(cropimg, (i[0], i[1]), int(i[2]/2), (255, 255, 255), 25)
        treasure.append([i[0],i[1]])
        #将treasure中的坐标值就近转换为25的倍数
        if i[0]%cell>cell/2:
            i[0] = i[0]+(cell-i[0]%cell)
        else:
            i[0] = i[0]-(i[0]%cell)
        if i[1]%cell>cell/2:
            i[1] = i[1]+(cell-i[1]%cell)
        else:
            i[1] = i[1]-(i[1]%cell)
        treasureinmap.append([int(i[0]/cell-3), int(i[1]/cell-1)])
    print("宝藏相对位置:",treasureinmap)

    return treasure,treasureinmap


def getBoard(imm, cropimg): #* 获取棋盘
    x, y, w, h = cv2.boundingRect(imm)
    print("棋盘尺寸 x,y,w,h:", x, y, w, h)
    board = [[0 for w in range(20)] for h in range(20)]  # 创建数组
    cell =int( h * 1.0/ 20);print("cellimm尺寸:",cell)
    for h in range(20):
        for w in range(20):
            if imm[cell * h+int(cell*1)-1, cell * w + int(cell * 3)-1] == np.uint8(255):  # 坐标转换
                board[h][w] = 255
    # print("board",board)
    # 数组以图像形式打印
    for h in range(20):
        for w in range(20):
            if board[h][w] == 255:
                cv2.rectangle(cropimg, (cell * w+int(cell * 2.5), cell * h+int(cell * 0.5)), (cell * w + int(cell * 2.5) + cell, cell * h + int(cell * 0.5) + cell), (0, 255, 0), 2)
    # cv2.imshow("BoardMap", cropimg)
    # cv2.waitKey(0)
    return board, cell
