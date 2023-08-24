import sys
import math
import cv2 as cv
import numpy as np
import time


def find_short_lines(img,lines, min_distance=40, max_distance=100, max_angle=5, min_line_length=100):
    short_lines = []
    long_lines = []
    if lines is None:
        pass
    else:
        for i in range(len(lines)):
            line = lines[i][0]
            #数组line的形式为[x1,y1,x2,y2]，求出线段的长度
            line_length = math.sqrt((line[2] - line[0]) ** 2 + (line[3] - line[1]) ** 2)
            if line_length > min_line_length:
                long_lines.append(line)
        for i in range(len(long_lines)):
            for j in range(i + 1, len(long_lines)):
                line1 = long_lines[i]
                line2 = long_lines[j]
                # 计算条之间的距离和夹角
                distance = min(math.sqrt((line2[0] - line1[0]) ** 2 + (line2[1] - line1[1]) ** 2),
                               math.sqrt((line2[2] - line1[0]) ** 2 + (line2[3] - line1[1]) ** 2),
                               math.sqrt((line2[0] - line1[2]) ** 2 + (line2[1] - line1[3]) ** 2),
                               math.sqrt((line2[2] - line1[2]) ** 2 + (line2[3] - line1[3]) ** 2))
                # 计算线段1的斜率
                slope1 = math.atan2(line1[3] - line1[1], line1[2] - line1[0])
                # 计算线段2的斜率
                slope2 = math.atan2(line2[3] - line2[1], line2[2] - line2[0])
                # 将斜率转换为角度，并获取正角度
                angle1 = math.degrees(slope1)
                angle2 = math.degrees(slope2)
                angle1 = abs(angle1)
                angle2 = abs(angle2)
                # 计算夹角差值
                angle = abs(angle1 - angle2)

                if min_distance < distance < max_distance and abs(angle) < max_angle:
                    short_lines.append((line1, line2))
                    endpoints1 = [
                        [[line1[0]], [line1[1]]],
                        [[line1[2]], [line1[3]]],
                        [[line2[0]], [line2[1]]],]
                    endpoints2 = [
                        [[line1[0]], [line1[1]]],
                        [[line1[2]], [line1[3]]],
                        [[line2[2]], [line2[3]]]]
                    endpoints3 = [
                        [[line1[0]], [line1[1]]],
                        [[line2[0]], [line2[1]]],
                        [[line2[2]], [line2[3]]]]
                    endpoints4 = [
                        [[line1[2]], [line1[3]]],
                        [[line2[0]], [line2[1]]],
                        [[line2[2]], [line2[3]]]]
                    cv.fillConvexPoly(img, np.array(endpoints1), (255, 255, 255))
                    cv.fillConvexPoly(img, np.array(endpoints2), (255, 255, 255))
                    cv.fillConvexPoly(img, np.array(endpoints3), (255, 255, 255))
                    cv.fillConvexPoly(img, np.array(endpoints4), (255, 255, 255))
    return img


def cardetect(img):
    src_color = img
    #src_color图像转换为灰度图像
    src = cv.cvtColor(src_color, cv.COLOR_BGR2GRAY)

    #开始计时
    # start = time.time()
    # Check if image is loaded fine
    if src is None:print('Error opening image!');return -1

    # 使用Canny检测器检测图像的边缘:
    dst = cv.Canny(src, 50, 200, None, 3)

    # Copy edges to the images that will display the results in BGR
    # cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    # cdstP = np.copy(cdst)
    # 概率Hough线变换
    # 首先应用转换:
    linesP = cv.HoughLinesP(dst, 1, np.pi / 360, 150, None, 50, 30)
    """
    dst:边缘检测器的输出。它应该是灰度图像(尽管实际上它是二值图像)
    lines:存储检测到的行参数(xstart、ystart、xend、yend)的向量
    rho:参数r的像素分辨率。我们使用1像素。
    θ:参数θ在弧度上的分辨率。我们用1度(CV_PI/180)
    阈值:“*检测*”一条直线的最小交点数
    minLineLength:可以组成一条直线的最小点数。少于这个点数的直线被忽略。
    maxLineGap:认为在同一直线上的两点之间的最大间隙。
    """
    # 寻找满足条件的短线
    last_img = find_short_lines(src_color, linesP, 2, 60, 10)
    #按百分比裁剪图像的x轴，y轴不变
    last_img = last_img[60:240, 40:200]
    # 将last_img分割成三个通道每个通道都转为灰度图
    b, g, r = cv.split(last_img)
    #使用otsu法对每个通道进行二值化
    ret, b = cv.threshold(b, 100, 255, cv.THRESH_BINARY)
    ret, g = cv.threshold(g, 100, 255, cv.THRESH_BINARY)
    ret, r = cv.threshold(r, 100, 255, cv.THRESH_BINARY)
    #统计图像中黑色像素的比例
    b_num = np.sum(b == 0) / (b.shape[0] * b.shape[1])
    g_num = np.sum(g == 0) / (g.shape[0] * g.shape[1])
    r_num = np.sum(r == 0) / (r.shape[0] * r.shape[1])
    # print(b_num, g_num, r_num)
    # end = time.time()
    #如果任意一个通道的黑色像素比例大于10%则为一个标志位赋值1否则赋值0
    if b_num > 0.1 or g_num > 0.1 or r_num > 0.1:
        colorflag = 1 # print("前方有车，撞他撞他！")
    else:
        colorflag = 0 # print("前方无车，安全通行！")
    return colorflag
