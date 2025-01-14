import math
from FindAllWays import reshape_image_find, ToBinray, GetGontours
from MapScan import reshape_image_scan, detect, find,affine_transformation
import cv2
import numpy as np
import time
import FindAllWays



# from configs import val_config
# from libs.detector.libs.detector.detector import Detector
# from utils import image_processing, debug, file_processing, torch_tools
# from models import inference
#
# project_root = os.path.dirname(__file__)

CAMERA_WIDTH = 1280;CAMERA_HEIGHT = 720 # 设定的相机取样像素参数

'''  类封装  '''
class Camera: # 相机调取类封装
    def __init__(self,camera):
        self.frame = []
        self.ret = False
        self.cap = 0
        self.camera = camera
    def open(self):
        self.cap = cv2.VideoCapture(self.camera)
        self.ret = self.cap.set(3, CAMERA_WIDTH)
        self.ret = self.cap.set(4, CAMERA_HEIGHT)
        self.cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret = False
        # threading.Thread(target=self.queryframe, args=()).start() # Thread-1 启动进程，清除CV2缓存序列-暂停使用
    def queryframe(self):
        self.ret, self.frame = self.cap.read()
    def read(self):
        self.ret, self.frame = self.cap.read()
        return self.ret, self.frame  # 返回读取结果和帧  # ret是布尔值，frame是图像数组



'''
    对象Map，主要有地图数据、起点和终点
'''
class Map(object):
    def __init__(self,mapdata,startx,starty,endx,endy):
        self.data = mapdata
        self.startx = startx
        self.starty = starty
        self.endx = endx
        self.endy = endy


'''
    Node.py主要是描述对象Node
'''
class Node(object):
    '''
        初始化节点信息
    '''
    def __init__(self,x,y,g,h,father):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.father = father
    '''
        处理边界和障碍点
    '''
    def getNeighbor(self,mapdata,endx,endy):
        x = self.x
        y = self.y
        result = []
    #先判断是否在上下边界
    #if(x!=0 or x!=len(mapdata)-1):
    #上
    #Node(x,y,g,h,father)
        if(x!=0 and mapdata[x-1][y]!=0):
            upNode = Node(x-1,y,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(upNode)
    #下
        if(x!=len(mapdata)-1 and mapdata[x+1][y]!=0):
            downNode = Node(x+1,y,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(downNode)
    #左
        if(y!=0 and mapdata[x][y-1]!=0):
            leftNode = Node(x,y-1,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(leftNode)
    #右
        if(y!=len(mapdata[0])-1 and mapdata[x][y+1]!=0):
            rightNode = Node(x,y+1,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(rightNode)
    #西北  14
        # if(x!=0 and y!=0 and mapdata[x-1][y-1]!=0 ):
        #     wnNode = Node(x-1,y-1,self.g+20,(abs(x-endx)+abs(y-endy))*10,self)
        #     result.append(wnNode)
    #东北
        # if(x!=0 and y!=len(mapdata[0])-1 and mapdata[x-1][y+1]!=0 ):
        #     enNode = Node(x-1,y+1,self.g+20,(abs(x-endx)+abs(y-endy))*10,self)
        #     result.append(enNode)
    #西南
        # if(x!=len(mapdata)-1 and y!=0 and mapdata[x+1][y-1]!=0 ):
        #     wsNode = Node(x+1,y-1,self.g+20,(abs(x-endx)+abs(y-endy))*10,self)
        #     result.append(wsNode)
    #东南
        # if(x!=len(mapdata)-1 and y!=len(mapdata[0])-1 and mapdata[x+1][y+1]!=0 ):
        #     esNode = Node(x+1,y+1,self.g+20,(abs(x-endx)+abs(y-endy))*10,self)
        #     result.append(esNode)
        # #如果节点在关闭节点 则不进行处理
        # finaResult = []
        # for i in result:
        #     if(i not in lockList):
        #         finaResult.append(i)
        # result = finaResult
        return result
    def hasNode(self,worklist):
        for i in worklist:
            if(i.x==self.x and i.y ==self.y):
                return True
        return False
    #在存在的前提下
    def changeG(self,worklist):
        for i in worklist:
            if(i.x==self.x and i.y ==self.y):
                if(i.g>self.g):
                    i.g = self.g


def getKeyforSort(element:Node):
    return element.g #element#不应该+element.h，


def astar(workMap):
    startx,starty = workMap.startx,workMap.starty
    endx,endy = workMap.endx,workMap.endy
    startNode = Node(startx, starty, 0, 0, None)
    openList = []
    lockList = []
    lockList.append(startNode)
    currNode = startNode
    while((endx,endy) != (currNode.x,currNode.y)):
        workList = currNode.getNeighbor(workMap.data,endx,endy)
        for i in workList:
            if (i not in lockList):
                if(i.hasNode(openList)):
                    i.changeG(openList)
                else:
                    openList.append(i)
        openList.sort(key=getKeyforSort)#关键步骤
        currNode = openList.pop(0)
        lockList.append(currNode)
    result = []
    while(currNode.father!=None):
        result.append((currNode.x,currNode.y))
        currNode = currNode.father
    result.append((currNode.x,currNode.y))
    return result


camera = Camera(0) # 打开相机
camera.open()
while 1:
    # ret, image = camera.read()
    '''照片扫描加纠偏部分'''
    image = cv2.imread('mapwithpoint1.jpg')
    # cv2.imshow("map", image)
    # cv2.waitKey(0)
    image, new_width, new_height = reshape_image_scan(image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image, contours, hierachy = detect(image)
    rec ,img= find(image, contours, np.squeeze(hierachy))
    img = affine_transformation(image, rec, 800, 800)
    # cv2.imshow("map", img)
    # cv2.waitKey(0)
    '''照片扫描加纠偏部分结束'''


    '''地图扫描输出数组部分'''
    # img = cv2.imread('map4.jpg')
    img, width, height = reshape_image_find(img)  # 调整图片大小
    # cv.imshow('img', img)
    ToBinray(img)  # 转二进制
    contours, boardx, cropimg, cell, treasureinmap = GetGontours(img)  # 提取轮廓
    for i in range(len(boardx)):  # 将board中255的值转换为1
        for j in range(len(boardx[0])):
            if(boardx[i][j]==255):
                boardx[i][j]=1
    boardmap = tuple(boardx)  # 将board转换为元组
    '''地图扫描输出数组部分结束'''

    for j in range(7):
        map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],treasureinmap[j+1][1],treasureinmap[j+1][0],)  # (mapdata,startx,starty,endx,endy)
        result = astar(map)
        result.reverse()
        print("怎么移动：", result)

        # x, y, w, h = cv2.boundingRect(cropimg)
        resultx = np.asarray(result)
        # print("result",resultx)
        # for h in range(20):
        #     for w in range(20):
        #         if resultx[h][w] == 1:
        #             cv2.rectangle(cropimg, (cell * w+int(cell * 3), cell * h+cell), (cell * w + int(cell * 3) + 2, cell * h + cell + 2), (0, 255, 0), 2)
        print("len",len(resultx))
        for i in range(len(resultx)):
            #等待0.3秒

            #绘制点
            # cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2, (255, 0, 0), 2)
            #绘制线
            if i != 0:
                cv2.line(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)),
                         (cell * resultx[i - 1][1] + int(cell * 3), cell * resultx[i - 1][0] + int(cell * 1)), (0, 0, 255), 2)
            cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2, (255, 0, 0), 2)

        # for i in range(len(resultx)):
        #     cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2,(255, 255, 255), 2)

        cv2.imshow("map", cropimg)
        cv2.waitKey(0)


    # cv2.destroyAllWindows()




