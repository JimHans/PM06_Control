#-*-coding:utf-8-*-
# Function: Main program
#? 主程序
#TODO Version 2.4.20230811
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
from MapScan_cy import reshape_image_scan, detect, find,affine_transformation, FindBlueOne  # 导入地图扫描部分
from Astar_cy import Map,astar,tsp # 导入A*算法部分
import Identify_cy # 导入识别模块
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
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < 5:
            ret, image = camera.read() # 读取相机图像
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE) # 旋转图像
            # 将相机图像显示在控件中
            qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0],image.shape[1]*3, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            self.lbl.setText('藏宝图调节时间剩余'+str(int(5-time.perf_counter() + start_time))+"秒，请注意")
            last_image = image

        camera.release()  # 释放相机资源
        # last_image = cv2.imread('./test/mapwithpoint1.jpg')# 读取test.jpg

        #? 照片扫描加纠偏部分开始
        time_calc = time.perf_counter()
        self.lbl.setText('照片扫描纠偏中...')
        image, new_width, new_height = reshape_image_scan(last_image)
        image, contours, hierachy = detect(image)
        rec ,img= find(image, contours, np.squeeze(hierachy))
        img = affine_transformation(image, rec, 800, 800)
        xblue, yblue = FindBlueOne(img)
        # 如果蓝色色块不在左下角或者右上角，就把图片旋转90度
        if xblue < 400 and yblue < 400:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif xblue > 400 and yblue > 400:
            img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0],img.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 照片扫描加纠偏部分结束

        #? 地图扫描输出数组部分开始
        self.lbl.setText('地图扫描输出进行中...')
        img, width, height = reshape_image_find(img)  # 调整图片大小
        _ , binary_img =ToBinray(img)  # 转二进制
        contours, boardx, cropimg, cell, treasureinmap = GetGontours(img, binary_img)  # 提取轮廓
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
        camID = find_available_camera()
        camera = Camera(camID) # 提前打开相机 
        camera.open()

        #TODO 运行8段路程，完成宝藏遍历
        for itel in range(len(result_final)):

            Performance_Calc = time.perf_counter() #! 路径规划性能运行时间计算
            #TODO 更新宝藏撞击后当前位置
            if Treasure_if_hittable == 1: # 如果上一个宝藏点可撞击,撞击后更新当前位置
                current_position[0] = TreasureRange[TreasureSequenceSaver-1][0][1]; current_position[1] = TreasureRange[TreasureSequenceSaver-1][0][0] # 撞击后更新当前位置

            #TODO 终点路径地图计算
            if True_Treas_Num == 3: #* 如果已经遍历了3个宝藏点，准备离开迷宫
                print('进入终点路径计算'); self.lbl.setText('运行中...前往终点'); final_point_route = 1 # 标记为最终路程
            else:self.lbl.setText('运行中...前往第'+str(TreasureSequenceSaver+1)+'个宝藏点')

            #TODO 动态路径规划(ASTAR)
            boardmap[TreasureRange[TreasureSequenceSaver][0][1]][TreasureRange[TreasureSequenceSaver][0][0]] = 1 # 将目标宝藏点设置为可走
            boardmap[current_position[0]][current_position[1]] = 1 # 将当前位置设置为可走
            prj_map = Map(boardmap, current_position[0],current_position[1],TreasureRange[TreasureSequenceSaver][0][1],TreasureRange[TreasureSequenceSaver][0][0])
            prj_map_result = astar(prj_map) # A*路径规划
            prj_map_result.reverse()
            prj_map_result = np.asarray(prj_map_result)
            route_points = prj_map_result

            for i in range(treasureinmapNum):boardmap[treasureinmap[i][1]][treasureinmap[i][0]] = 2 # 设定所有宝藏点为特殊点位
            # if 1: # if TreasureSequenceSaver==0 or final_point_route == 1: #静态预计算路径,已停用
            # else:
            #     boardmap[TreasureRange[TreasureSequenceSaver][0][1]][TreasureRange[TreasureSequenceSaver][0][0]] = 1 # 将目标宝藏点设置为可走
            #     boardmap[current_position[0]][current_position[1]] = 1 # 将当前位置设置为可走
            #     prj_map_result = Treas_Route_Matrix[int(TreasureRange[TreasureSequenceSaver-1][1])][int(TreasureRange[TreasureSequenceSaver][1])] # 读取预计算宝藏点路径
            #     prj_map_result.reverse()
            #     prj_map_result = np.asarray(prj_map_result)
            #     route_points = prj_map_result

            #TODO 计算本段路径拐点
            corners = [] ; print('本段路径总长度(仮):',(len(route_points) - 1))
            for j in range(1,len(route_points) - 1,1):
                dx = route_points[j+1][0] - route_points[j-1][0]
                dy = route_points[j+1][1] - route_points[j-1][1]
                delta = abs(dx * dx) + abs(dy * dy)
                if delta == 2 or (route_points[j][0]==10 and route_points[j][1]==6) or (route_points[j][0]==8 and route_points[j][1]==12):corners.append(route_points[j]) 
            corners.append(route_points[-1]) # 补上最终目标点

            print("路径动态规划Astar耗时:",time.perf_counter() - Performance_Calc) #! 路径规划性能运行时间显示
            Performance_Calc = time.perf_counter() #! 路径发送性能运行时间计算

            #TODO 将路段数目发给下位机
            Full_route_numbers = len(corners)+1 if Treasure_if_hittable == 1 else len(corners)  # 若本次需要撞击宝藏,路段数目+1
            RouteNumberCalc = 1 if Treasure_if_hittable == 1 else 0 # 路段数目计算存储
            if corners[0][0] == current_position[0] and corners[0][1] == current_position[1]: Full_route_numbers-=1 # 如果首段路径为0(一般出现在不撞击宝藏时，车坐标正好处于路口情况下),不发送首段路径
            #* 如果上一个宝藏点可撞击,在下一次运动路径中注入撞击指令[路段数目+1]
            serialapi.communicate(0xaa,0xc1,eval(hex(Full_route_numbers)),0x00,0x00,0x00,0x00)
            while True:
                if str(serialapi.recv)[0:14] == 'aa01c100000000': break;# 等待接收到回复信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            self.sublbl.setText('运行中...前往第'+str(TreasureSequenceSaver+1)+'个宝藏点,路段数目'+str(Full_route_numbers)+'已发送') # self.sublbl.setText("前往宝藏点"+str(TreasureSequenceSaver+1)+"路段数目为"+str(Full_route_numbers))
            print("当前位置",str(current_position),"前往宝藏点",str(TreasureSequenceSaver+1),"路段数目为"+str(Full_route_numbers))
            
            #TODO 将撞击指令单独发给下位机
            if Treasure_if_hittable == 1: # 如果上一个宝藏点可撞击,在下一次运动路径中注入撞击指令
                print("运动第1段路径开始啦")# 发送控制指令
                serialapi.communicate(0xaa,0xc2,0x00,eval(hex(Treasure_hit_route[0])),eval(hex(Treasure_hit_route[1])),eval(hex(Treasure_hit_route[2]+2)),eval(hex(0)))
                # while True:
                #     if str(serialapi.recv)[0:14] == 'aa01c200000000': break;# 等待接收到回复信号
                serialapi.recv = str.encode('xxxxxxxxxxx')
                self.sublbl.setText('运行中...前往第'+str(TreasureSequenceSaver+1)+'个宝藏点前置上一宝藏撞击指令已发送') 
                print("运动第1段路径前往",[TreasureRange[TreasureSequenceSaver-1][0][1],TreasureRange[TreasureSequenceSaver-1][0][0]],"拐点,前进方向",Treasure_hit_route[0],"测距方向",Treasure_hit_route[1],"弧距0挡板距离",Treasure_hit_route[2]+2)

            #TODO 计算并发送每段路程控制指令并等待运行完成
            for index,corner in enumerate(corners):
                #* 计算拐点与当前位置的距离
                dx2 = corner[0] - current_position[0];dy2 = corner[1] - current_position[1]
                if (index+1)<len(corners): # 如果不是最后一个拐点
                    dx2_future = corners[index+1][0]-corner[0];dy2_future = corners[index+1][1]-corner[1]
                    #* 计算转向弧距
                    TempArcMarker = 0 #* 临时记录弧线距离(默认为0)
                    if dy2_future>0:
                        while True: #! 计算转向弧距
                            if not((corner[1]-TempArcMarker)>=0 and boardmap[corner[0]][corner[1]-TempArcMarker] == 1):break
                            TempArcMarker += 1
                    elif dy2_future<0:
                        while True: #! 计算转向弧距
                            if not((TempArcMarker+corner[1])<19 and boardmap[corner[0]][TempArcMarker+corner[1]] == 1):break
                            TempArcMarker += 1
                    elif dx2_future>0:
                        while True: #! 计算转向弧距
                            if not((corner[0]-TempArcMarker)>=0 and boardmap[corner[0]-TempArcMarker][corner[1]] == 1):break
                            TempArcMarker += 1
                    elif dx2_future<0:
                        while True: #! 计算转向弧距
                            if not((TempArcMarker+corner[0])<19 and boardmap[TempArcMarker+corner[0]][corner[1]] == 1):break
                            TempArcMarker += 1
                else: # 如果是最后一个拐点
                    dx2_future = -1919810;dy2_future = -1919810;TempArcMarker = 1
                
                If_Treasure_On_The_Way = 0 #* 检测宝藏点是否在目标前进方向上
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
                
                print("运动第"+str(RouteNumberCalc+1)+"段路径开始啦")
                current_position = list(corner) # 更新当前位置

                #TODO 接近宝藏时保持与宝藏距离防止撞到，并纠正当前坐标
                if index+1 == len(corners): # 最后一段路程
                    single_route_steps -= 2;TempMarker -=2 # 少走一步防止撞到宝藏,挡板距离也少2
                    if TempMarker <1:TempMarker = 1 # 挡板距离小于0，修正为0
                    Treasure_hit_route = [Direction,SensDirection,TempMarker-1] # 记录宝藏撞击路程的方向与距挡板距离[撞击需要]
                    if Direction == 0:current_position[1] -=2 # 修正向右当前坐标
                    elif Direction == 1:current_position[0] +=2 # 修正向上当前坐标
                    elif Direction == 2:current_position[1] +=2 # 修正向左当前坐标
                    elif Direction == 3:current_position[0] -=2 # 修正向下当前坐标 
                #* 若是已完成8个宝藏遍历,或者已是前往出口路程,则在该路程最后一段子路径前进方向上多走4步,走出迷宫
                if (TreasureSequenceSaver+1 ==len(result_final) and index+1 == len(corners)) or (index+1 == len(corners) and final_point_route == 1): 
                    single_route_steps = 4;TempMarker = 5  # 出口路程固定为4步，冲出去
                if index+1 == len(corners) and single_route_steps <1: # 每大段路程最后一段，若步数小于1，前进方向与上一段测距方向相反
                    Direction = 2 if last_SensDirection == 0 else (3 if last_SensDirection == 1 else (0 if last_SensDirection == 2 else 1))
                    SensDirection +=4 # 同时测距方向+4
                else:last_SensDirection = SensDirection # 若不是每大段路程最后一段，记录本次路程的测距方向供之后使用
                if TempMarker <1:TempMarker = 1 # 挡板距离小于0，修正为0

                #TODO 发送控制指令
                print("运动第"+str(RouteNumberCalc+1)+"段路径前往"+str(corner)+"拐点,前进方向",Direction,"测距方向",SensDirection,"弧距",TempArcMarker,"挡板距离",TempMarker-1)
                if not (dx2==0 and dy2==0): # 仅发送不为0的路径
                    serialapi.communicate(0xaa,0xc2,eval(hex(RouteNumberCalc)),eval(hex(Direction)),eval(hex(SensDirection)),eval(hex(TempMarker-1)),eval(hex(TempArcMarker-1)))
                    # while True:
                    #     if str(serialapi.recv)[0:14] == 'aa01c2'+'{:02x}'.format(RouteNumberCalc)+'000000': break;# 等待接收到回复信号
                    serialapi.recv = str.encode('xxxxxxxxxxx');RouteNumberCalc+=1 # 路径段数+1 
                else:print("运动第"+str(RouteNumberCalc+1)+"段路径因距离为0跳过")
                self.sublbl.setText('运行中...前往第'+str(TreasureSequenceSaver+1)+'个宝藏点,路段'+str(index+1)+'已发送')
            
            print("路径发送耗时:",time.perf_counter() - Performance_Calc) #! 路径发送运行时间显示
            Performance_Calc = time.perf_counter() #! 路径运行时间计算
            #TODO 等待运行完成发送回复指令
            while True:
                if str(serialapi.recv)[0:14] == 'aa210000000000': break;# 等待接收到回复识别请求信号
                if STOP_Thread == 1:
                    break# 如果接收到停止信号,break
            if STOP_Thread == 1:break# 如果接收到停止信号,break
            serialapi.recv = str.encode('xxxxxxxxxxx')
            serialapi.communicate(0xaa,0x01,0x21,0x00,0x00,0x00,0x00)
            print("第"+str(TreasureSequenceSaver+1)+"个宝藏点已到达,当前位置",current_position)
            self.lbl.setText("第"+str(TreasureSequenceSaver+1)+"个宝藏点已到达,当前位置"+str(current_position))
            Treasure_if_hittable = 0 # 重置宝藏撞击标志位

            #TODO 如果已经是最终路程，退出循环
            if final_point_route == 1:break
            
            print("路径运行耗时:",time.perf_counter() - Performance_Calc) #! 路径运行时间显示
            # TODO 拍照识别
            if TreasureFinishList[TreasureRange[TreasureSequenceSaver][1]] == 1: # 预测过的宝藏不再识别
                print("调用预测数据，真宝藏...")
                # TODO 调用预测结果并判断是否发送撞击指令
                if car_color_group == "BLUE":
                    Treasure = "BlueTrue"
                    Treasure_if_hittable = 1 # 宝藏点可撞击
                    self.sublbl.setText("可以撞击，已注入撞击指令")
                    print("可以撞击，已注入撞击指令")
                    True_Treas_Num += 1 # 真宝藏数量+1
                elif car_color_group == "RED":
                    Treasure == "RedTrue"
                    Treasure_if_hittable = 1 # 宝藏点可撞击
                    self.sublbl.setText("可以撞击，已注入撞击指令")
                    print("可以撞击，已注入撞击指令")
                    True_Treas_Num += 1 # 真宝藏数量+1
            else:
                self.sublbl.setText("正在进行拍摄识别...")
                cam_index = 0
                while cam_index<3:
                    ret, Treas_image = camera.read() # 读取相机宝藏图像
                    if ret:cam_index+=1
                cv2.imwrite("./imgsave/Treas_image"+str(int(time.time()))+".jpg",Treas_image)
                Performance_Calc = time.perf_counter() #! 识别性能运行时间计算
                Treas_img_copy = Treas_image.copy()
                Treas_image = reshape_image_scan(Treas_image)[0]
                Treas_image, contours, yellow = Identify_cy.FindColorOne(Treas_img_copy, 1)  # 黄色
                Treas_image, contours, green = Identify_cy.FindColorOne(Treas_img_copy, 2)  # 绿色
                Treas_image, contours, blue = Identify_cy.FindColorOne(Treas_img_copy, 0)  # 蓝色
                Treas_image, contours, red = Identify_cy.FindRedOne(Treas_img_copy, contours)  # 红色
                #蓝色1，黄色1，为蓝色真宝藏；红色1，绿色1，为红色真宝藏；蓝色1，绿色1，为蓝色假宝藏；红色1，黄色1，为红色假宝藏
                if blue == 1 and yellow == 1:
                    self.sublbl.setText('蓝色真宝藏');print("蓝色真宝藏");Treasure = "BlueTrue"
                elif red == 1 and green == 1:
                    self.sublbl.setText('红色真宝藏');print("红色真宝藏");Treasure = "RedTrue"
                elif blue == 1 and green == 1:
                    self.sublbl.setText('蓝色假宝藏');print("蓝色假宝藏");Treasure = "BlueFalse"
                elif red == 1 and yellow == 1:
                    self.sublbl.setText('红色假宝藏');print("红色假宝藏");Treasure = "RedFalse"
                else:
                    self.sublbl.setText('无宝藏');print("无宝藏");Treasure = "None"

                # TODO 根据识别结果判断是否发送撞击指令
                if car_color_group == "BLUE":
                    if Treasure == "BlueTrue":
                        Treasure_if_hittable = 1 # 宝藏点可撞击
                        self.sublbl.setText("可以撞击，已注入撞击指令")
                        print("可以撞击，已注入撞击指令")
                        True_Treas_Num += 1 # 真宝藏数量+1
                    else:
                        Treasure_if_hittable = 0 # 宝藏点不可撞击
                        print("不可以撞击，未注入撞击指令")
                elif car_color_group == "RED":
                    if Treasure == "RedTrue":
                        Treasure_if_hittable = 1 # 宝藏点可撞击
                        self.sublbl.setText("可以撞击，已注入撞击指令")
                        print("可以撞击，已注入撞击指令")
                        True_Treas_Num += 1 # 真宝藏数量+1
                    else:
                        Treasure_if_hittable = 0 # 宝藏点不可撞击
                        print("不可以撞击，未注入撞击指令")
            # self.camera_label.setPixmap(map_gui_image)

            print("宝藏识别耗时:",time.perf_counter() - Performance_Calc) #! 宝藏识别运行时间显示
            Performance_Calc = time.perf_counter() #! 路径点动态规划运行时间计算

            # TODO 根据识别结果动态规划下个目标点,存储宝藏点信息[0为未识别,1为预识别为真,-1为已撞击/遍历我方真宝藏,-2为已撞击/遍历对方真宝藏,-3为已撞击/遍历双方假宝藏(小于0均认为不用前往),-4为对方宝藏(不确定),2为同色待定]
            if Treasure == "BlueFalse" or Treasure == "RedFalse":TreasureFinishList[TreasureRange[TreasureSequenceSaver][1]] = -3 # 假宝藏标记
            elif (car_color_group == "BLUE" and Treasure == "RedTrue") or (car_color_group == "RED" and Treasure == "BlueTrue"):TreasureFinishList[TreasureRange[TreasureSequenceSaver][1]] = -2 # 对方真宝藏标记
            else: TreasureFinishList[TreasureRange[TreasureSequenceSaver][1]] = -1 # 标记已经到达的我方真宝藏为已撞击/遍历
            cv2.putText(map_cv2_image, str(TreasureSequenceSaver+1), (cell * TreasureRange[TreasureSequenceSaver][0][0] + int(cell * 2), cell * TreasureRange[TreasureSequenceSaver][0][1] + int(cell * 1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA, False)
            #? 对称宝藏优化
            for i in range(treasureinmapNum): # 遍历所有宝藏点寻找中心对称宝藏
                if TreasureFinishList[i] ==0 or TreasureFinishList[i] ==2: # 仅更新未撞击/遍历的宝藏点,不覆盖先前推测结果和预识别结果
                    if treasureinmap_ori[i][1] == 18-TreasureRange[TreasureSequenceSaver][0][1] and treasureinmap_ori[i][0] == 18-TreasureRange[TreasureSequenceSaver][0][0]:
                        if Treasure_if_hittable ==1: TreasureFinishList[i] = -2 #* 如果宝藏点可撞击,标记已经到达的宝藏点中心对称点为对方真宝藏(已撞击/遍历)
                        else: # 如果宝藏点不可撞击
                            if (car_color_group == "RED" and Treasure == "BlueTrue") or (car_color_group == "BLUE" and Treasure == "RedTrue"): # 如果是对方真宝藏
                                TreasureFinishList[i] = 1 #* 标记已经到达的宝藏点中心对称点为己方真宝藏(待撞击)
                            if (Treasure == "RedFalse" or Treasure == "BlueFalse"): # 如果是假宝藏
                                TreasureFinishList[i] = -3 #* 标记已经到达的宝藏点中心对称点为对方假宝藏(已撞击/遍历)
            #? 同象限宝藏优化
            for i in range(treasureinmapNum): # 遍历所有宝藏点寻找同象限宝藏
                if TreasureFinishList[i] ==0 or TreasureFinishList[i] ==2: # 仅更新未撞击/遍历的宝藏点,不覆盖先前推测结果
                    if ((treasureinmap_ori[i][0]-9)/abs(treasureinmap_ori[i][0]-9) == (TreasureRange[TreasureSequenceSaver][0][0]-9)/abs(TreasureRange[TreasureSequenceSaver][0][0]-9)) and (
                                (treasureinmap_ori[i][1]-9)/abs(treasureinmap_ori[i][1]-9) == (TreasureRange[TreasureSequenceSaver][0][1]-9)/abs(TreasureRange[TreasureSequenceSaver][0][1]-9)):
                        if (car_color_group == "RED" and (Treasure == "RedTrue" or Treasure == "RedFalse")) or (car_color_group == "BLUE" and (Treasure == "BlueTrue" or Treasure == "BlueFalse")):
                            TreasureFinishList[i] = -4 #* 若本次为我方颜色宝藏，标记同象限不同色宝藏点为对方不确定宝藏(已撞击/遍历)
                        elif (car_color_group == "RED" and (Treasure == "BlueTrue" or Treasure == "BlueFalse")) or (car_color_group == "BLUE" and (Treasure == "RedTrue" or Treasure == "RedFalse")):
                            TreasureFinishList[i] = 2 #* 若本次为对方颜色宝藏，标记同象限不同色宝藏点为己方未确定宝藏
            #? 同象限宝藏的对称宝藏优化
            for i in range(treasureinmapNum): # 遍历所有宝藏点寻找同象限宝藏
                if ((treasureinmap_ori[i][0]-9)/abs(treasureinmap_ori[i][0]-9) == (TreasureRange[TreasureSequenceSaver][0][0]-9)/abs(TreasureRange[TreasureSequenceSaver][0][0]-9)) and (
                            (treasureinmap_ori[i][1]-9)/abs(treasureinmap_ori[i][1]-9) == (TreasureRange[TreasureSequenceSaver][0][1]-9)/abs(TreasureRange[TreasureSequenceSaver][0][1]-9)):
                    if (car_color_group == "RED" and (Treasure == "RedTrue" or Treasure == "RedFalse")) or (car_color_group == "BLUE" and (Treasure == "BlueTrue" or Treasure == "BlueFalse")):
                        for j in range(treasureinmapNum): # 当本次为我方宝藏,同象限宝藏为对方宝藏时,遍历所有宝藏点寻找同象限宝藏的对称点
                            if 18-treasureinmap_ori[i][0] == treasureinmap_ori[j][0] and 18-treasureinmap_ori[i][1] == treasureinmap_ori[j][1]:
                                if TreasureFinishList[j] ==0 or TreasureFinishList[j] ==2:TreasureFinishList[j] = 2 #* 若本次为我方颜色宝藏，标记同象限不同色宝藏点的对称点为己方未确定宝藏
                    elif (car_color_group == "RED" and (Treasure == "BlueTrue" or Treasure == "BlueFalse")) or (car_color_group == "BLUE" and (Treasure == "RedTrue" or Treasure == "RedFalse")):
                        for j in range(treasureinmapNum): # 当本次为对方宝藏,同象限宝藏为我方宝藏时,遍历所有宝藏点寻找同象限宝藏的对称点
                            if 18-treasureinmap_ori[i][0] == treasureinmap_ori[j][0] and 18-treasureinmap_ori[i][1] == treasureinmap_ori[j][1]:
                                if TreasureFinishList[j] ==0 or TreasureFinishList[j] ==2:TreasureFinishList[j] = -4 #* 若本次为对方颜色宝藏，标记同象限不同色宝藏点的对称点为对方不确定宝藏(已撞击/遍历)
            #? 假宝藏优化
            if Treasure == "BlueFalse" or Treasure == "RedFalse" or False_Treasure_Found == 1: # 如果本次是假宝藏或已经发现假宝藏
                for i in range(10): # 遍历所有宝藏点寻找假宝藏
                    if TreasureFinishList[i] ==2:TreasureFinishList[i] =1;False_Treasure_Found=1 #* 标记未确定宝藏为己方真宝藏(待撞击)
            #? 动态计算下一目的地
            Total_Visited_Treasure = 0 # 已经遍历的宝藏点数量
            for i in range(10): #* 检查是否所有点都去过 并更新显示数据
                if TreasureFinishList[i] < 0: 
                    Total_Visited_Treasure += 1 # 计算总遍历宝藏数目
                    if TreasureFinishList[i] == -2: # 对方真宝藏
                        if car_color_group == "RED": 
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (255, 0, 0), 24)
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 255), 12)
                        else:
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 0, 255), 24)
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 0), 12)
                    if TreasureFinishList[i] == -3: # 假宝藏
                        cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0,255, 255), 24)
                        cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 0), 12)
                    if TreasureFinishList[i] == -4: # 对方不确定宝藏
                        if car_color_group == "RED":cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (255, 0, 0), 24)
                        else:cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 0, 255), 24)
                    if TreasureFinishList[i] == -1: # 已撞击宝藏
                        if car_color_group == "RED":
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 0, 255), 24)
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 0), 12)
                        else:
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (255, 0, 0), 24)
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 255), 12)
                    if TreasureFinishList[i] == 1: # 预识别己方真宝藏
                        if car_color_group == "RED": 
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 0, 255), 24)
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 0), 12)
                        else:
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (255, 0, 0), 24)
                            cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 255, 255), 12)
                    if TreasureFinishList[i] == 2: # 预识别我方待定宝藏
                        if car_color_group == "RED": cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (0, 0, 255), 24)
                        else:cv2.circle(map_cv2_image, (cell * treasureinmap_ori[i][0] + int(cell * 3), cell * treasureinmap_ori[i][1] + int(cell * 1)), 2, (255, 0, 0), 24)
            if Treasure=='None' and TreasureFinishList[TreasureRange[TreasureSequenceSaver][1]] == -1: # 无宝藏
                cv2.circle(map_cv2_image, (cell * TreasureRange[TreasureSequenceSaver][0][0] + int(cell * 3), cell * TreasureRange[TreasureSequenceSaver][0][1] + int(cell * 1)), 2, (0, 0, 0), 24)
    
            if True_Treas_Num == 3 or TreasureSequenceSaver+2 ==len(result_final) or Total_Visited_Treasure==8: 
                TreasureRange.append([[18,0],10])
                print("经计算，下个目标点是终点") 
                final_point_route = 1 # 若已经完成三个寻宝或8次运动,将终点存入队列
            else:
                # 对数组从小到大排序,计算最近点并加入队列
                Temp_Next_Treasure = [1000,1000] # 临时变量,存储下一个宝藏点
                for i in range(treasureinmapNum): # 遍历所有宝藏点寻找下一运动点
                    if TreasureFinishList[i] >= 0: # 找出未遍历的点
                        if(Treas_distances[TreasureRange[TreasureSequenceSaver][1]][i] < Temp_Next_Treasure[0]): # 找出距离最近的点
                            Temp_Next_Treasure[0] = Treas_distances[TreasureRange[TreasureSequenceSaver][1]][i];Temp_Next_Treasure[1] = i # 存储距离最近的点
                if Temp_Next_Treasure != [1000,1000]: TreasureRange.append([treasureinmap_ori[Temp_Next_Treasure[1]],Temp_Next_Treasure[1]]) # 下一个宝藏点存入队列
                print("经计算，下个目标宝藏点是：",[TreasureRange[TreasureSequenceSaver+1][0][1],TreasureRange[TreasureSequenceSaver+1][0][0]])
            
            for i in range(treasureinmapNum):
                if TreasureFinishList[i]==-1:boardmap[treasureinmap_ori[i][1]][treasureinmap_ori[i][0]] = 1 # 屏蔽除已撞击我方宝藏以外的所有宝藏点不能走
                else:boardmap[treasureinmap_ori[i][1]][treasureinmap_ori[i][0]] = 0

            qimg = QtGui.QImage(map_cv2_image.data, map_cv2_image.shape[1], map_cv2_image.shape[0],map_cv2_image.shape[1]*3, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)

            print("路径点动态规划推算耗时:",time.perf_counter() - Performance_Calc) #! 路径点动态规划推算运行时间显示
            TreasureSequenceSaver += 1 # 宝藏点遍历计数+1

        # TODO 断点终止
        if STOP_Thread == 1:
            print("断点续传触发成功,先前运行已终止");camera.release();self.lbl.setText("断点续传启动，运行已终止，下次预期前往"+str(TreasureSequenceSaver+1)+"号宝藏点");
            return 0 # 如果接收到停止信号,退出程序
        # TODO 到达终点
        self.sublbl.setText('Misson Complete');self.lbl.setText('完成运行,耗时'+str(int(time.perf_counter() - calctimer))+'秒')
        camera.release()  # 释放相机资源
        # pixmap = QtGui.QPixmap('layla.png')
        # 在 QLabel 控件中显示图片
        # self.camera_label.setPixmap(pixmap)
        # self.bgmPlayer.stop() # 停止播放音乐


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())