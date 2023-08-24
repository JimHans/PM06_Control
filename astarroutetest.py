#-*-coding:utf-8-*-
# Function: Main program
#? 主程序
#TODO Version 3.0.20230821
#! 依赖项目：PyQt5 | OpenCV | FindAllWays.py | MapScan | Astar.py | Identify.py | serialapi.py | networkx | itertools
#* Thread 利用情况：Thread-0 UART通信
#* Thread 利用情况：Thread-1 路径规划线程
#* Thread 利用情况：Thread-2 地图扫描线程
import sys,cv2,time,threading # 导入系统模块,OpenCV模块
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QInputDialog # 导入PyQt5模块
from PyQt5 import QtGui,QtCore # 导入PyQt5GUI模块
from PyQt5.QtCore import QUrl # 导入PyQt5多媒体模块
from PyQt5.QtMultimedia import QMediaPlayer,QMediaContent # 导入PyQt5多媒体模块
from FindAllWays import reshape_image_find, ToBinray, GetGontours # 导入地图寻路部分
from MapScan_cy import reshape_image_scan, detect, find,affine_transformation, FindBlueOne, FindRedOne   # 导入地图扫描部分
from Astar_cy import Map,astar,tsp # 导入A*算法部分
import Identify_cy # 导入识别模块
from cardetect import cardetect # 导入对方车识别模块
import numpy as np # 导入Numpy模块
import serialapi # 导入串口模块

CAMERA_WIDTH = 1280;CAMERA_HEIGHT = 1080  # 设定的相机取样像素参数

result_final = []                         # 寻路结果存储
car_color_group = 'RED'                   # 小车颜色存储，默认为红色
map_gui_image = None ;map_cv2_image = None# 地图路径规划完成图像存储
cell = None                               # 地图单元格大小
True_Treas_Num = 0                        # 已遍历真实宝藏点数目
# 宝藏点距离,宝藏点数据,路径预计算数据存储
Treas_distances = [] ;treasureinmap_ori = [];Treas_Route_Matrix = []
#! 断点续传全局运行数据存储
STOP_Thread = 0             # 线程停止标志位
Startup_Times = 0           # 重复启动次数
RunCarThreadVar = None      # 小车正式运行线程变量
final_point_route = 0       # 最终路程标记
False_Treasure_Found = 0    # 是否发现假宝藏
TreasureFinishList = [0]*10 # 宝藏点完成情况记录
treasureinmapNum = 8        # 宝藏点总数
TreasureRange = []          # 宝藏点撞击顺序集合
TreasureSequenceSaver = 0   # 宝藏点撞击No存储

def find_available_camera():
    for i in range(256):
        cap = cv2.VideoCapture(i)
        ret, frame = cap.read()
        if cap.isOpened() and ret:
            print("Camera index:", i, "is available.")
            cap.release()
            return i
    print("No available camera found.")
    return None

'''  类封装  '''
class Camera: #TODO 相机调取类封装
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
    def release(self):
        self.cap.release()

class Example(QWidget): #TODO 主窗口类


    def __init__(self): #* 初始化函数
        super().__init__()

        self.initUI()


    def initUI(self): #* 界面初始化函数
        global car_color_group # 全局变量调用
        self.lbl = QLabel('系统初始化进行中', self)
        self.lbl.move(0, 10)
        self.lbl.setFixedSize(600, 30)
        self.lbl.setAlignment(QtCore.Qt.AlignCenter) # 修改字体居中
        self.lbl.setFont(QtGui.QFont("Arial", 16)) # 修改字体大小为16px

        self.sublbl = QLabel(' 等待路径规划完成后运行', self)
        self.sublbl.move(0, 40)
        self.sublbl.setFixedSize(600, 30)
        self.sublbl.setAlignment(QtCore.Qt.AlignCenter) # 修改字体居中
        self.sublbl.setFont(QtGui.QFont("Arial", 12)) # 修改字体大小为12px

        # 启动按钮
        self.stbtn = QPushButton('拍摄藏宝图', self)
        self.stbtn.clicked.connect(self.startup_thread) # 修改按钮行为为启动特定函数“Startup”
        self.stbtn.move(40, 80)
        self.stbtn.setFixedSize(160, 50)
        # 仿真按钮
        self.simbtn = QPushButton(self)
        self.simbtn.setGeometry(40, 150, 160, 50)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(simunready.png);}")
        self.simbtn.setFixedSize(72, 72)
        self.simbtn.clicked.connect(self.simulate_thread) # 修改按钮行为为启动特定函数“Simulate”
        self.simbtn.setEnabled(False)
        # 运行按钮
        self.runbtn = QPushButton('启动AutoCar运行', self)
        self.runbtn.clicked.connect(self.runcar_thread) # 修改按钮行为为启动小车运行函数
        self.runbtn.move(220, 80)
        self.runbtn.setFixedSize(180, 50)
        self.runbtn.setEnabled(False)
        # 断点运行按钮
        self.btn = QPushButton('断点运行', self)
        self.btn.clicked.connect(self.stop_thread)
        self.btn.move(420, 80)
        self.btn.setFixedSize(140, 50)

        self.setGeometry(100, 40, 600, 400)
        self.setWindowTitle('CARSYS')
        self.setWindowIcon(QtGui.QIcon('icon.png'))
        self.setFont(QtGui.QFont("Arial", 16)) # 修改字体大小为16px

        # 创建QMediaPlayer对象
        # self.bgmPlayer = QMediaPlayer()
        # 加载音乐文件
        # MusicUrl = QUrl.fromLocalFile("/home/rock/Desktop/PM06Master/bgm.mp3")
        # MusicContent = QMediaContent(MusicUrl)
        # self.bgmPlayer.setMedia(MusicContent)

        # 添加运行中对方车识别系统显示画面
        self.cardetectshow = QLabel(self)
        self.cardetectshow.move(500, 150)
        self.cardetectshow.setFixedSize(72, 72)
        self.cardetectshow.setScaledContents(True)
        # 添加一个用于显示相机图像的控件
        self.camera_label = QLabel(self)
        self.camera_label.move(130, 150)
        self.camera_label.setFixedSize(340, 240)
        self.camera_label.setScaledContents(True)
        # 加载一张图片
        pixmap = QtGui.QPixmap('layla.png')
        # 在 QLabel 控件中显示图片
        self.camera_label.setPixmap(pixmap)
        self.show()

        # 系统串口检测
        self.lbl.setText('检测串口通信通路')
        if serialapi.If_Serial_Open == False: # 检测串口是否打开
            self.lbl.setText('串口通信通路异常,系统初始化失败')
        else:
            threading.Thread(target=serialapi.uartRx, args=()).start() #* Thread-0 开启串口接收子线程
            serialapi.communicate(0xaa,0xa1,0x00,0x00,0x00,0x00,0x00) # 发送启动信号
            start_time = time.perf_counter();Serial_response=0
            while time.perf_counter() - start_time < 8:
                if str(serialapi.recv)[0:14] == 'aa01a100000000': 
                    serialapi.recv = str.encode('xxxxxxxxxxx')
                    Serial_response=1
                    self.lbl.setText('串口通信通路正常,系统初始化完成')
                    break;# 等待接收到启动信号
            if Serial_response==0:self.lbl.setText('串口通信通路异常,系统初始化失败')
        
        # 选择阵营
        car_color_group, ok = QInputDialog.getItem(None, '选择颜色', '请选择当前是红方还是蓝方：', ['RED', 'BLUE'])
        self.sublbl.setText('当前阵营为：'+car_color_group+",等待路径规划完成后运行")


    def startup_thread(self): #* 启动路径规划线程
        t = threading.Thread(target=self.Startup)
        t.start()


    def simulate_thread(self): #* 启动仿真线程
        t = threading.Thread(target=self.Simulate)
        t.start()


    def runcar_thread(self): #* 启动运行线程
        global RunCarThreadVar
        RunCarThreadVar = threading.Thread(target=self.Runcar)
        RunCarThreadVar.start()


    def stop_thread(self): #* 停止运行线程
        global STOP_Thread,RunCarThreadVar,Startup_Times
        STOP_Thread = 1 # 线程停止标志位
        Startup_Times+=1 # 重复启动次数+1
        # self.lbl.setText("断点续传启动，运行已终止")
        # RunCarThreadVar.stop()# 如果接收到停止信号，关闭本线程


    def Startup(self): #* 启动函数
        global result_final,boardmap,map_gui_image,cell,map_cv2_image,treasureinmap,Treas_distances,treasureinmap_ori,Treas_Route_Matrix
        camID = find_available_camera()
        camera = Camera(camID) # 打开相机
        camera.open()
        # 等待调整相机并拍照
        last_image = None
        # start_time = time.perf_counter()
        while True:
            ret, image = camera.read()  # 读取相机图像
            last_image = image
            qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0], image.shape[1] * 3,
                                QtGui.QImage.Format_BGR888) # 将图像转换为QImage格式
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            self.lbl.setText("扫描图片获取定位点中...")
            # ? 照片扫描加纠偏部分开始
            image, contours, hierachy = detect(image) # 检测图像中的轮廓
            rec, img = find(image, contours, np.squeeze(hierachy)) # 找到图像中的矩形轮廓,并返回矩形轮廓的四个顶点

            if len(rec) == 4:  # 找到四个定位点后标志位置位并进行下一步
                rec_flag = 1
                img = affine_transformation(image, rec, 800, 800) # 透视变换
                xblue, yblue, area = FindBlueOne(img) # 找到蓝色色块，并强制将蓝色色块放到左下角
                # 如果蓝色色块在左上角，将图片顺时针旋转90度
                if xblue < 400 and yblue < 400:
                    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                # 如果蓝色色块在右下角，将图片逆时针旋转90度
                elif xblue > 400 and yblue > 400:
                    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                # 如果蓝色色块在右上角，将图片旋转180度
                elif xblue > 400 and yblue < 400:
                    img = cv2.rotate(img, cv2.ROTATE_180)
                # cv2.imwrite('./mapspinned.jpg', img)
                # 找到旋转后的蓝色和红色色块，定位出迷宫的角点
                # todo 注意！此处红蓝色块阈值需要根据实际情况调整
                xblue, yblue, bluearea = FindBlueOne(img)
                xred, yred, redarea = FindRedOne(img)
                cellx = abs(xblue-xred)/22
                celly = abs(yblue-yred)/18
                cellcrop = (cellx + celly) / 2
                # print("cellcrop:", cellcrop)
                x_leftdown = xblue - cellcrop
                y_leftdown = yblue + cellcrop
                x_rightup = xred + cellcrop
                y_rightup = yred - cellcrop

                # 将图像按照两个坐标点进行裁剪
                imgx = img[int(y_rightup):int(y_leftdown), int(x_leftdown):int(x_rightup)]
                # cv2.imwrite('./mapcrop.jpg', imgx)
                # ? 照片扫描加纠偏部分结束

                # ? 地图扫描输出数组部分开始
                self.lbl.setText('地图扫描输出进行中...')
                # 将图片的大小强制调整为600*500
                imgx = cv2.resize(imgx, (600, 500), interpolation=cv2.INTER_CUBIC)
                boardx, img, cell, treasureinmap = GetGontours(imgx,imgx)  # 获取宝藏图中路径信息，宝藏信息，以及每个格子的大小

                cropimg = imgx # cv2.cvtColor(imgx, cv2.COLOR_BGR2RGB)  # 将img转换为QImage格式

                qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0], cropimg.shape[1] * 3, QtGui.QImage.Format_BGR888)  # 将img转换为QImage格式
                pixmap = QtGui.QPixmap.fromImage(qimg)
                self.camera_label.setPixmap(pixmap)
                # ? 地图扫描输出数组部分结束

                if len(treasureinmap) == 8:
                    trea_flag = 1
                else:
                    trea_flag = 0
                    self.sublbl.setText("宝藏点不够,需要调整相机")
            else:
                rec_flag = 0
                self.sublbl.setText("定位点不足4个,重新拍照")
            if rec_flag == 1 and trea_flag == 1:
                self.sublbl.setText('地图扫描完成!')
                break

        # while time.perf_counter() - start_time < 5:
        #     ret, image = camera.read() # 读取相机图像
        #     image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE) # 旋转图像
        #     # 将相机图像显示在控件中
        #     qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0],image.shape[1]*3, QtGui.QImage.Format_BGR888)
        #     pixmap = QtGui.QPixmap.fromImage(qimg)
        #     self.camera_label.setPixmap(pixmap)
        #     self.lbl.setText('藏宝图调节时间剩余'+str(int(5-time.perf_counter() + start_time))+"秒，请注意")
        #     last_image = image

        camera.release()  # 释放相机资源
        # last_image = cv2.imread('./test/mapwithpoint1.jpg')# 读取test.jpg

        #? 照片扫描加纠偏部分开始
        time_calc = time.perf_counter()
        # self.lbl.setText('照片扫描纠偏中...')
        # image, new_width, new_height = reshape_image_scan(last_image)
        # image, contours, hierachy = detect(image)
        # rec ,img= find(image, contours, np.squeeze(hierachy))
        # img = affine_transformation(image, rec, 800, 800)
        # xblue, yblue = FindBlueOne(img)
        # # 如果蓝色色块不在左下角或者右上角，就把图片旋转90度
        # if xblue < 400 and yblue < 400:
        #     img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        # elif xblue > 400 and yblue > 400:
        #     img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # # 将图像显示在QT控件中
        # qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0],img.shape[1]*3, QtGui.QImage.Format_BGR888)
        # pixmap = QtGui.QPixmap.fromImage(qimg)
        # self.camera_label.setPixmap(pixmap)
        #? 照片扫描加纠偏部分结束

        #? 地图扫描输出数组部分开始
        # self.lbl.setText('地图扫描输出进行中...')
        # img, width, height = reshape_image_find(img)  # 调整图片大小
        # _ , binary_img =ToBinray(img)  # 转二进制
        # contours, boardx, cropimg, cell, treasureinmap = GetGontours(img, binary_img)  # 提取轮廓
        for i in range(len(boardx)):  # 将board中255的值转换为1
            for j in range(len(boardx[0])):
                if(boardx[i][j]==255):boardx[i][j]=1
        boardmap = tuple(boardx)  # 将board转换为元组
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 地图扫描输出数组部分结束

        self.lbl.setText('路径规划进行中...')
        treasureinmap_ori = treasureinmap # 保存原始的宝藏坐标
        treasureinmap,Treas_distances,Treas_Route_Matrix = tsp(boardmap,treasureinmap) # 调用tsp算法
        result_final = []
        for j in range(-1,8,1):
            if j==-1: map = Map(boardmap, 18,0,treasureinmap[j+1][1],treasureinmap[j+1][0],)
            elif j==7: map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],0,18,)
            else: map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],treasureinmap[j+1][1],treasureinmap[j+1][0],)  # (mapdata,startx,starty,endx,endy)
            result = astar(map) # 调用astar算法
            result.reverse()
            resultx = np.asarray(result)
            for i in range(len(resultx)): # 绘制线段
                if i != 0:
                    cv2.line(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)),
                            (cell * resultx[i - 1][1] + int(cell * 3), cell * resultx[i - 1][0] + int(cell * 1)), (0, 0, 255), 2)
                cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2, (255, 0, 0), 2)
            result_final.append(result)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        map_gui_image = pixmap
        map_cv2_image = cropimg
        self.camera_label.setPixmap(pixmap)
        for i in range(len(treasureinmap)):boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 0 # 屏蔽所有宝藏点不能走
        # 界面更新
        self.stbtn.setEnabled(False); self.runbtn.setEnabled(True); self.simbtn.setEnabled(True)
        self.lbl.setText('路径规划完成,耗时{:.2f}s'.format(time.perf_counter()-time_calc)+',可以进行路径模拟或按下Enter运行...')
        self.sublbl.setText('等待指令...')
        self.simbtn.setStyleSheet("QPushButton { border-image: url(sim.png);}")


    def Simulate(self): #* 仿真函数
        global result_final,boardmap,cell,map_cv2_image
        self.simbtn.setEnabled(False)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(simunready.png);}") # 按钮状态修改为不可执行，等待模拟完成后解除
        self.lbl.setText('模拟系统启动，正在模拟运行...')
        sim_timer = time.perf_counter() # 计时器开始
        for index,router in enumerate(result_final):
            temp_img = map_cv2_image.copy()
            self.sublbl.setText('正在前往目标点'+str(index+1)+'...')
            for j in range(len(router)): # 绘制线段
                cv2.circle(temp_img, (cell * router[j][1] + int(cell * 3), cell * router[j][0] + int(cell * 1)), 2, (0, 0, 255), 6)
                if j != 0:
                    cv2.line(temp_img, (cell * router[j][1] + int(cell * 3), cell * router[j][0] + int(cell * 1)),
                            (cell * router[j - 1][1] + int(cell * 3), cell * router[j - 1][0] + int(cell * 1)), (255, 0, 0), 6)
                cv2.circle(temp_img, (cell * router[j][1] + int(cell * 3), cell * router[j][0] + int(cell * 1)), 2, (0, 0, 255), 6)
                if(j%3==1): 
                    cv2.circle(temp_img, (cell * router[-1][1] + int(cell * 3), cell * router[-1][0] + int(cell * 1)), 2, (0, 0, 255), 24)
                    cv2.circle(temp_img, (cell * router[0][1] + int(cell * 3), cell * router[0][0] + int(cell * 1)), 2, (50, 255, 0), 24)
                else: 
                    cv2.circle(temp_img, (cell * router[-1][1] + int(cell * 3), cell * router[-1][0] + int(cell * 1)), 2, (255, 0, 0), 24)
                    cv2.circle(temp_img, (cell * router[0][1] + int(cell * 3), cell * router[0][0] + int(cell * 1)), 2, (255, 255, 255), 24)
                # 将图像显示在QT控件中
                qimg = QtGui.QImage(temp_img.data, temp_img.shape[1], temp_img.shape[0],temp_img.shape[1]*3, QtGui.QImage.Format_BGR888)
                pixmap = QtGui.QPixmap.fromImage(qimg)
                self.camera_label.setPixmap(pixmap)
                time.sleep(0.2) # 模拟运行延时
        self.lbl.setText('模拟运行完成，预计耗时'+str(int(time.perf_counter()-sim_timer))+'秒')
        self.simbtn.setEnabled(True)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(sim.png);}")


    def Runcar(self): #* 运行函数
        global result_final,boardmap,car_color_group,map_cv2_image,True_Treas_Num,treasureinmap,treasureinmap_ori,Treas_distances# ,Treas_Route_Matrix
        calctimer = time.perf_counter() # 计时器开始,播放音乐 self.bgmPlayer.play()
        current_position = [18,0] # 当前位置存储
        Treasure_hit_route = [0,0,0] # 宝藏点撞击方向/距挡板距离集合
        Treasure_if_hittable = 0 # 宝藏点是否可撞击
        Treasure = "None" # 宝藏点标记
        map_cv2_image_show = map_cv2_image.copy() #* 复制地图用于显示

        #! 断点续传全局运行数据设定
        global STOP_Thread,RunCarThreadVar,Startup_Times,final_point_route,False_Treasure_Found,TreasureFinishList,treasureinmapNum,TreasureRange,TreasureSequenceSaver
        STOP_Thread = 0 # 线程停止标志位重置
        if Startup_Times == 0: # 如果是第一次启动,初始化全局变量
            final_point_route = 0 # 最终路程标记
            False_Treasure_Found = 0 # 是否发现假宝藏
            TreasureFinishList = [0]*10 # 宝藏点完成情况记录
            treasureinmapNum = len(treasureinmap_ori)
            TreasureRange = [[treasureinmap[0],0]] # 宝藏点撞击顺序集合,提前写入第一个宝藏点位置
            for i in range(len(treasureinmap_ori)): # 遍历所有宝藏点寻找中心对称宝藏
                if treasureinmap_ori[i][1] == treasureinmap[0][1] and treasureinmap_ori[i][0] == treasureinmap[0][0]:
                    TreasureRange[0][1] = i;break # 找到第一个宝藏点的原始位置
        camera = None # 创建相机对象

        #TODO 运行8段路程，完成宝藏遍历
        Performance_Calc = time.perf_counter() #! 路径规划性能运行时间计算

        #TODO 动态路径规划(ASTAR)
        boardmap[TreasureRange[TreasureSequenceSaver][0][1]][TreasureRange[TreasureSequenceSaver][0][0]] = 1 # 将目标宝藏点设置为可走
        boardmap[current_position[0]][current_position[1]] = 1 # 将当前位置设置为可走
        prj_map = Map(boardmap, current_position[0],current_position[1],current_position[0],current_position[1]+2)
        prj_map_result = astar(prj_map) # A*路径规划
        prj_map_result.reverse()
        prj_map_result = np.asarray(prj_map_result)
        route_points = prj_map_result

        for i in range(treasureinmapNum):boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 2 # 设定所有宝藏点为特殊点位

        #TODO 计算本段路径拐点
        corners = [] ; print('本段路径总长度(仮):',(len(route_points) - 1))
        for j in range(1,len(route_points) - 1,1):
            dx = route_points[j+1][0] - route_points[j-1][0]
            dy = route_points[j+1][1] - route_points[j-1][1]
            delta = abs(dx * dx) + abs(dy * dy)
            if delta == 2 or (route_points[j][0]==10 and route_points[j][1]==6) or (route_points[j][0]==8 and route_points[j][1]==12):corners.append(route_points[j]) 
        corners.append(route_points[-1]) # 补上最终目标点

        #TODO 计算并发送每段路程控制指令并等待运行完成
        for index,corner in enumerate(corners):
            If_Treasure_On_The_Way = 0 #* 检测宝藏点是否在目标前进方向上
            #* 计算拐点与当前位置的距离
            dx2 = corner[0] - current_position[0];dy2 = corner[1] - current_position[1]
            
            #* 地图向右运行相关参数计算
            if dy2 >0: # 向右
                Direction = 0;SensDirection = 0 if index+1 < len(corners) else 2;single_route_steps = dy2
                if (current_position[0]==10 and current_position[1]<6) or (
                        current_position[0]==8 and current_position[1]<12 and current_position[1]>=6):SensDirection = 2
                TempMarker = 0 # 临时记录挡板距离
                if SensDirection == 2:
                    while True: #! 检测距挡板距离-最后路径段
                        if not((corner[1]-TempMarker)>=0 and (boardmap[corner[0]][corner[1]-TempMarker] == 1 or boardmap[corner[0]][corner[1]-TempMarker] == 2)):break
                        TempMarker += 1
                else:
                    while True: # 检测距挡板距离-普通路径段
                        if not((TempMarker+corner[1])<19 and boardmap[corner[0]][TempMarker+corner[1]] == 1):
                            if boardmap[corner[0]][TempMarker+corner[1]] == 2:
                                If_Treasure_On_The_Way = 1
                            break
                        TempMarker += 1
                    if If_Treasure_On_The_Way == 1:
                        SensDirection = 2 # 修正测距方向
                        TempMarker = 0 # 重置先前计算的挡板距离
                        while True: #! 检测距挡板距离-前方有宝藏时
                            if not((corner[1]-TempMarker)>=0 and boardmap[corner[0]][corner[1]-TempMarker] == 1):break
                            TempMarker += 1
            #* 地图向左运行相关参数计算
            elif dy2 <0: # 向左
                Direction = 2;SensDirection = 2 if index+1 < len(corners) else 0;single_route_steps = abs(dy2)
                if (current_position[0]==8 and current_position[1]>12) or (
                        current_position[0]==10 and current_position[1]<=12 and current_position[1]>6):SensDirection = 0
                TempMarker = 0 # 临时记录挡板距离
                if SensDirection == 0:
                    while True: #! 检测距挡板距离-最后路径段
                        if not((TempMarker+corner[1])<19 and (boardmap[corner[0]][TempMarker+corner[1]] == 1 or boardmap[corner[0]][TempMarker+corner[1]] == 2)):break
                        TempMarker += 1
                else:
                    while True: # 检测距挡板距离
                        if not((corner[1]-TempMarker)>=0 and boardmap[corner[0]][corner[1]-TempMarker] == 1):
                            if boardmap[corner[0]][corner[1]-TempMarker] == 2:
                                If_Treasure_On_The_Way = 1
                            break
                        TempMarker += 1
                    if If_Treasure_On_The_Way == 1:
                        SensDirection = 0 # 修正测距方向
                        TempMarker = 0 # 重置先前计算的挡板距离
                        while True: #! 检测距挡板距离-前方有宝藏时
                            if not((TempMarker+corner[1])<19 and boardmap[corner[0]][TempMarker+corner[1]] == 1):break
                            TempMarker += 1
            #* 地图向上运行相关参数计算
            elif dx2 <0: # 向上
                Direction = 1;SensDirection = 1 if index+1 < len(corners) else 3;single_route_steps = abs(dx2)
                TempMarker = 0 # 临时记录挡板距离
                if SensDirection == 3:
                    while True: #! 检测距挡板距离-最后路径段
                        if not((TempMarker+corner[0])<19 and (boardmap[TempMarker+corner[0]][corner[1]] == 1 or boardmap[TempMarker+corner[0]][corner[1]] == 2)):break
                        TempMarker += 1
                else:
                    while True: # 检测距挡板距离
                        if not((corner[0]-TempMarker)>=0 and boardmap[corner[0]-TempMarker][corner[1]] == 1):
                            if boardmap[corner[0]-TempMarker][corner[1]] == 2:
                                If_Treasure_On_The_Way = 1
                            break
                        TempMarker += 1
                    if If_Treasure_On_The_Way == 1:
                        SensDirection = 3 # 修正测距方向
                        TempMarker = 0 # 重置先前计算的挡板距离
                        while True: #! 检测距挡板距离-前方有宝藏时
                            if not((TempMarker+corner[0])<19 and boardmap[TempMarker+corner[0]][corner[1]] == 1):break
                            TempMarker += 1
            #* 地图向下运行相关参数计算
            else: # 向下
                Direction = 3;SensDirection = 3 if index+1 < len(corners) else 1;single_route_steps = dx2
                TempMarker = 0 # 临时记录挡板距离
                if SensDirection == 1:
                    while True: #! 检测距挡板距离-最后路径段
                        if not((corner[0]-TempMarker)>=0 and (boardmap[corner[0]-TempMarker][corner[1]] == 1 or boardmap[corner[0]-TempMarker][corner[1]] == 2)):break
                        TempMarker += 1
                else:
                    while True: # 检测距挡板距离
                        if not((TempMarker+corner[0])<19 and boardmap[TempMarker+corner[0]][corner[1]] == 1):
                            if boardmap[TempMarker+corner[0]][corner[1]] == 2:
                                If_Treasure_On_The_Way = 1
                            break
                        TempMarker += 1
                    if If_Treasure_On_The_Way == 1:
                        SensDirection = 1 # 修正测距方向
                        TempMarker = 0 # 重置先前计算的挡板距离
                        while True: #! 检测距挡板距离-前方有宝藏时
                            if not((corner[0]-TempMarker)>=0 and boardmap[corner[0]-TempMarker][corner[1]] == 1):break
                            TempMarker += 1
            
            current_position = list(corner) # 更新当前位置

            #TODO 发送控制指令
            print("运动第段路径前往"+str(corner)+"拐点,前进方向",Direction,"测距方向",SensDirection,"挡板距离",TempMarker-1)
            self.sublbl.setText('运行中...前往第'+str(TreasureSequenceSaver+1)+'个宝藏点,路段'+str(index+1)+'已发送')

        # TODO 到达终点
        self.sublbl.setText('Misson Complete');self.lbl.setText('完成运行,耗时'+str(int(time.perf_counter() - calctimer))+'秒')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())