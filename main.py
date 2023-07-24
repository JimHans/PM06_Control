#-*-coding:utf-8-*-
# Function: Main program
#? 主程序
#TODO Version 0.12.20230725
#! 依赖项目：PyQt5 | OpenCV | FindAllWays.py | MapScan | Astar.py | Identify.py | serialapi.py | networkx | itertools
#* Thread 利用情况：Thread-0 UART通信
#* Thread 利用情况：Thread-1 路径规划线程
#* Thread 利用情况：Thread-2 地图扫描线程
import sys,cv2,time,threading # 导入系统模块,OpenCV模块
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QInputDialog # 导入PyQt5模块
from PyQt5 import QtGui,QtCore # 导入PyQt5GUI模块
from FindAllWays import reshape_image_find, ToBinray, GetGontours # 导入地图寻路部分
from MapScan import reshape_image_scan, detect, find,affine_transformation # 导入地图扫描部分
from Astar_cy import Map,astar,tsp # 导入A*算法部分
import Identify # 导入识别模块
import numpy as np # 导入Numpy模块
import serialapi # 导入串口模块
# import networkx as nx # 导入网络图模块
# import itertools # 导入迭代器模块

CAMERA_WIDTH = 1280;CAMERA_HEIGHT = 720 # 设定的相机取样像素参数
result_final = [] # 寻路结果存储
car_color_group = 'RED' # 小车颜色存储，默认为红色
map_gui_image = None ;map_cv2_image = None# 地图路径规划完成图像存储
cell = None # 地图单元格大小

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
        # 退出按钮
        self.btn = QPushButton('退出程序', self)
        self.btn.clicked.connect(QApplication.instance().quit)
        self.btn.move(420, 80)
        self.btn.setFixedSize(140, 50)

        self.setGeometry(100, 40, 600, 400)
        self.setWindowTitle('CARSYS')
        self.setWindowIcon(QtGui.QIcon('icon.png'))
        self.setFont(QtGui.QFont("Arial", 16)) # 修改字体大小为16px

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
            start_time = time.time();Serial_response=0
            while time.time() - start_time < 8:
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
        t = threading.Thread(target=self.Runcar)
        t.start()


    def Startup(self): #* 启动函数
        global result_final,boardmap,map_gui_image,cell,map_cv2_image
        camera = Camera(5) # 打开相机
        camera.open()
        # 等待调整相机并拍照
        last_image = None
        start_time = time.time()
        while time.time() - start_time < 5:
            ret, image = camera.read() # 读取相机图像
            # 将相机图像显示在控件中
            qimg = QtGui.QImage(image.data, image.shape[1], image.shape[0],image.shape[1]*3, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.camera_label.setPixmap(pixmap)
            self.lbl.setText('调节时间剩余'+str(int(5-time.time() + start_time))+"秒，请注意")
            last_image = image

        camera.release()  # 释放相机资源
        last_image = cv2.imread('./test/mapwithpoint1.jpg')# 读取test.jpg

        #? 照片扫描加纠偏部分开始
        time_calc = time.time()
        self.lbl.setText('照片扫描纠偏中...')
        image, new_width, new_height = reshape_image_scan(last_image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image, contours, hierachy = detect(image)
        rec ,img= find(image, contours, np.squeeze(hierachy))
        img = affine_transformation(image, rec, 800, 800)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0],img.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 照片扫描加纠偏部分结束

        #? 地图扫描输出数组部分开始
        self.lbl.setText('地图扫描输出进行中...')
        img, width, height = reshape_image_find(img)  # 调整图片大小
        ToBinray(img)  # 转二进制
        contours, boardx, cropimg, cell, treasureinmap = GetGontours(img)  # 提取轮廓
        for i in range(len(boardx)):  # 将board中255的值转换为1
            for j in range(len(boardx[0])):
                if(boardx[i][j]==255):
                    boardx[i][j]=1
        boardmap = tuple(boardx)  # 将board转换为元组
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(pixmap)
        #? 地图扫描输出数组部分结束

        self.lbl.setText('路径规划进行中...')
        treasureinmap = tsp(boardmap,treasureinmap) # 调用tsp算法
        # '''tsp部分'''
        # # 计算起点(18,0)到各个宝藏的距离
        # firstdistance = []
        # for i in range(len(treasureinmap)):
        #     map = Map(boardmap, 18, 0, treasureinmap[i][1], treasureinmap[i][0])
        #     result = astar(map)
        #     np.append(firstdistance,len(result) - 1)
        # # print("firstdistance:", firstdistance)
        # # 将距离起点最近的宝藏放在第一个
        # for i in range(len(firstdistance)):
        #     if firstdistance[i] == min(firstdistance):
        #         temp = treasureinmap[0]
        #         treasureinmap[0] = treasureinmap[i]
        #         treasureinmap[i] = temp
        # # print("treasureinmap1:", treasureinmap)
        # # 计算各个宝藏之间的距离
        # distances = [[0 for i in range(8)] for j in range(8)]
        # for i in range(8):
        #     for j in range(8):
        #         if i == j:
        #             distances[i][j] = 0
        #         else:
        #             map = Map(boardmap, treasureinmap[i][1], treasureinmap[i][0], treasureinmap[j][1],
        #                         treasureinmap[j][0])
        #             result = astar(map)
        #             distances[i][j] = len(result) - 1
        # # print("距离矩阵distance:", distances)
        # # 停止计时
        # #print("计算距离矩阵用时：", end - start)
        # # 计算最短路径
        # # 定义城市和距离矩阵
        # cities = [1, 2, 3, 4, 5, 6, 7, 8]
        # # 创建完全图
        # G = nx.Graph()
        # G.add_nodes_from(cities)
        # for i, j in itertools.combinations(range(len(cities)), 2):
        #     G.add_edge(cities[i], cities[j], weight=distances[i][j])

        # # 求解旅行商问题
        # shortest_tour = None
        # min_tour_length = float('inf')
        # for permutation in itertools.permutations(cities):
        #     tour_length = sum([G[permutation[i]][permutation[i + 1]]['weight'] for i in range(len(cities) - 1)])
        #     tour_length += G[permutation[-1]][permutation[0]]['weight']
        #     if tour_length < min_tour_length:
        #         shortest_tour = permutation
        #         min_tour_length = tour_length
        # print("最短路径：", shortest_tour, "总距离成本",min_tour_length)

        # order = [0] * 8
        # for i in range(8):
        #     order[i] = shortest_tour[i]-1
        # # 将最优线路中的宝藏按顺序进行重新排列
        # temp_treasure_map = [0]*8
        # for index,orderget in enumerate(order):
        #     temp_treasure_map[index] = treasureinmap[orderget]
        # treasureinmap = temp_treasure_map
        # print("最优线路:", treasureinmap)
        # '''tsp部分结束'''
        result_final = []
        for j in range(-1,8,1):
            if j==-1: map = Map(boardmap, 18,0,treasureinmap[j+1][1],treasureinmap[j+1][0],)
            elif j==7: map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],0,18,)
            else: map = Map(boardmap, treasureinmap[j][1],treasureinmap[j][0],treasureinmap[j+1][1],treasureinmap[j+1][0],)  # (mapdata,startx,starty,endx,endy)
            result = astar(map)
            result.reverse()
            resultx = np.asarray(result)
            # print("len",len(resultx))
            for i in range(len(resultx)): # 绘制线段
                if i != 0:
                    cv2.line(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)),
                            (cell * resultx[i - 1][1] + int(cell * 3), cell * resultx[i - 1][0] + int(cell * 1)), (0, 0, 255), 2)
                cv2.circle(cropimg, (cell * resultx[i][1] + int(cell * 3), cell * resultx[i][0] + int(cell * 1)), 2, (255, 0, 0), 2)
            result_final.append(result)
        # print("怎么移动：", result_final)
        # 将图像显示在QT控件中
        qimg = QtGui.QImage(cropimg.data, cropimg.shape[1], cropimg.shape[0],cropimg.shape[1]*3, QtGui.QImage.Format_BGR888)
        pixmap = QtGui.QPixmap.fromImage(qimg)
        map_gui_image = pixmap
        map_cv2_image = cropimg
        self.camera_label.setPixmap(pixmap)
        # 界面更新
        self.stbtn.setEnabled(False)
        self.runbtn.setEnabled(True)
        self.simbtn.setEnabled(True)
        self.lbl.setText('路径规划完成,耗时{:.2f}s'.format(time.time()-time_calc)+',可以进行路径模拟或按下Enter运行...')
        self.simbtn.setStyleSheet("QPushButton { border-image: url(sim.png);}")


    def Simulate(self): #* 仿真函数
        global result_final,boardmap,cell,map_cv2_image
        self.simbtn.setEnabled(False)
        self.simbtn.setStyleSheet("QPushButton { border-image: url(simunready.png);}") # 按钮状态修改为不可执行，等待模拟完成后解除
        self.lbl.setText('模拟系统启动，正在模拟运行...')
        sim_timer = time.time() # 计时器开始
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
        self.lbl.setText('模拟运行完成，预计耗时'+str(int(time.time()-sim_timer))+'秒')
        self.simbtn.setEnabled(True)


    def Runcar(self): #* 运行函数
        global result_final,boardmap,car_color_group,map_gui_image
        current_position = [18,-2] # 当前位置存储
        Treasure_hit_route = [0,0,0] # 宝藏点撞击方向/距挡板距离集合
        Treasure_if_hittable = 0 # 宝藏点是否可撞击
        calctimer = time.time() # 计时器开始

        #TODO 运行8段路程，完成宝藏遍历
        for itel in range(len(result_final)):
            self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点')

            #TODO 计算拐点
            route_points = result_final[itel]
            corners = []
            print(len(route_points) - 1)
            for j in range(1,len(route_points) - 1,1):
                dx = route_points[j+1][0] - route_points[j-1][0]
                dy = route_points[j+1][1] - route_points[j-1][1]
                delta = abs(dx * dx) + abs(dy * dy)
                if delta == 2:corners.append(route_points[j])
                else:continue
            corners.append(route_points[-1]) # 补上最终目标点

            #TODO 将路段数目发给下位机
            Full_route_numbers = len(corners)+2 if Treasure_if_hittable == 1 else len(corners) 
            # 如果上一个宝藏点可撞击,在下一次运动路径中注入撞击指令[路段数目+2]
            serialapi.communicate(0xaa,0xc1,eval(hex(Full_route_numbers)),0x00,0x00,0x00,0x00)
            while True:
                if str(serialapi.recv)[0:14] == 'aa01c100000000': break;# 等待接收到回复信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点,路段数目'+str(Full_route_numbers)+'已发送') 
            self.sublbl.setText("前往宝藏点"+str(itel+1)+"路段数目为"+str(Full_route_numbers))
            print("前往宝藏点",str(itel+1),"路段数目为"+str(Full_route_numbers))

            if Treasure_if_hittable == 1: # 如果上一个宝藏点可撞击,在下一次运动路径中注入撞击指令
                # 发送控制指令
                serialapi.communicate(0xaa,0xc2,0x00,eval(hex(Treasure_hit_route[0])),eval(hex(Treasure_hit_route[1])),eval(hex(Treasure_hit_route[2]+2)),eval(hex(2)))
                while True:
                    if str(serialapi.recv)[0:14] == 'aa01c200000000': break;# 等待接收到回复信号
                serialapi.recv = str.encode('xxxxxxxxxxx')
                self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点前置上一宝藏撞击指令1已发送') 

                # time.sleep(0.1) # 等待0.1s
                serialapi.communicate(0xaa,0xc2,0x01,eval(hex(Treasure_hit_route[1])),eval(hex(Treasure_hit_route[1])),eval(hex(Treasure_hit_route[2])),eval(hex(2)))
                while True:
                    if str(serialapi.recv)[0:14] == 'aa01c201000000': break;# 等待接收到回复信号
                serialapi.recv = str.encode('xxxxxxxxxxx')
                self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点前置上一宝藏脱离指令2已发送') 

            #TODO 计算并发送每段路程控制指令并等待运行完成
            for index,corner in enumerate(corners):
                # 计算拐点与当前位置的距离
                dx2 = corner[0] - current_position[0];dy2 = corner[1] - current_position[1]
                if dy2 >0: # 向右
                    Direction = 0;SensDirection = 0 if index+1 < len(corners) else 2;single_route_steps = dy2
                    TempMarker = 0 # 临时记录挡板距离
                    if index+1 == len(corners):
                        while True: #! 检测距挡板距离-最后路径段
                            if not((corner[1]-TempMarker)>=0 and boardmap[corner[0]][corner[1]-TempMarker] == 1):break
                            TempMarker += 1
                    else:
                        while True: # 检测距挡板距离-普通路径段
                            if not((TempMarker+corner[1])<19 and boardmap[corner[0]][TempMarker+corner[1]] == 1):break
                            TempMarker += 1
                elif dy2 <0: # 向左
                    Direction = 2;SensDirection = 2 if index+1 < len(corners) else 0;single_route_steps = abs(dy2)
                    TempMarker = 0 # 临时记录挡板距离
                    if index+1 == len(corners):
                        while True: #! 检测距挡板距离-最后路径段
                            if not((TempMarker+corner[1])<19 and boardmap[corner[0]][TempMarker+corner[1]] == 1):break
                            TempMarker += 1
                    else:
                        while True: # 检测距挡板距离
                            if not((corner[1]-TempMarker)>=0 and boardmap[corner[0]][corner[1]-TempMarker] == 1):break
                            TempMarker += 1
                elif dx2 <0: # 向上
                    Direction = 1;SensDirection = 1 if index+1 < len(corners) else 3;single_route_steps = abs(dx2)
                    TempMarker = 0 # 临时记录挡板距离
                    if index+1 == len(corners):
                        while True: #! 检测距挡板距离-最后路径段
                            if not((TempMarker+corner[0])<19 and boardmap[TempMarker+corner[0]][corner[1]] == 1):break
                            TempMarker += 1
                    else:
                        while True: # 检测距挡板距离
                            if not((corner[0]-TempMarker)>=0 and boardmap[corner[0]-TempMarker][corner[1]] == 1):break
                            TempMarker += 1
                elif dx2 >0: # 向下
                    Direction = 3;SensDirection = 3 if index+1 < len(corners) else 1;single_route_steps = dx2
                    TempMarker = 0 # 临时记录挡板距离
                    if index+1 == len(corners):
                        while True: #! 检测距挡板距离-最后路径段
                            if not((corner[0]-TempMarker)>=0 and boardmap[corner[0]-TempMarker][corner[1]] == 1):break
                            TempMarker += 1
                    else:
                        while True: # 检测距挡板距离
                            if not((TempMarker+corner[0])<19 and boardmap[TempMarker+corner[0]][corner[1]] == 1):break
                            TempMarker += 1
                
                print("实际运动第"+str(index+1)+"段路径开始啦")
                current_position = list(corner) # 更新当前位置
                if index+1 == len(corners): # 最后一段路程
                    single_route_steps -= 2 # 少走一步防止撞到宝藏
                    TempMarker -=2 # 挡板距离也少2
                    Treasure_hit_route = [Direction,SensDirection,TempMarker-1] # 记录宝藏撞击路程的方向与距挡板距离[如果需要撞击]
                    if Direction == 0:current_position[1] -=2 # 修正向右当前坐标
                    elif Direction == 1:current_position[0] +=2 # 修正向上当前坐标
                    elif Direction == 2:current_position[1] +=2 # 修正向左当前坐标
                    elif Direction == 3:current_position[0] -=2 # 修正向下当前坐标 
                if itel+1 ==9 and index+1 == len(corners): # 前往出口路程
                    single_route_steps = 4  # 出口路程固定为4步，冲出去
                    TempMarker = 5 # 出口路程固定为5步，冲出去

                print("实际运动第"+str(index+1)+"段路径前往"+str(corner)+"拐点,前进方向",Direction,"测距方向",SensDirection,
                                                                "路径长",single_route_steps,"挡板距离",TempMarker-1)
                # 发送控制指令
                if Treasure_if_hittable == 1:
                    serialapi.communicate(0xaa,0xc2,eval(hex(index+2)),eval(hex(Direction)),eval(hex(SensDirection)),eval(hex(TempMarker-1)),eval(hex(single_route_steps)))
                    while True:
                        if str(serialapi.recv)[0:14] == 'aa01c2'+'{:02x}'.format(index+2)+'000000': break;# 等待接收到回复信号
                else:
                    serialapi.communicate(0xaa,0xc2,eval(hex(index)),eval(hex(Direction)),eval(hex(SensDirection)),eval(hex(TempMarker-1)),eval(hex(single_route_steps)))
                    while True:
                        if str(serialapi.recv)[0:14] == 'aa01c2'+'{:02x}'.format(index)+'000000': break;# 等待接收到回复信号
                serialapi.recv = str.encode('xxxxxxxxxxx')
                self.lbl.setText('运行中...前往第'+str(itel+1)+'个宝藏点,路段'+str(index+1)+'已发送') 
                # time.sleep(0.1)
            
            #TODO 等待运行完成
            while True:
                if str(serialapi.recv)[0:14] == 'aa210000000000': break;# 等待接收到回复识别请求信号
            serialapi.recv = str.encode('xxxxxxxxxxx')
            #TODO 发送回复指令
            serialapi.communicate(0xaa,0x01,0x21,0x00,0x00,0x00,0x00)
            print("第"+str(itel+1)+"个宝藏点已到达,当前位置",current_position)
            self.sublbl.setText("第"+str(itel+1)+"个宝藏点已到达,当前位置"+str(current_position))
            Treasure_if_hittable = 0 # 重置宝藏撞击标志位
            # time.sleep(1)

            # TODO 拍照识别
            camera = Camera(5) # 打开相机
            camera.open()
            for k in range(10):
                ret, Treas_image = camera.read() # 读取相机宝藏图像
                Treas_qimg = QtGui.QImage(Treas_image.data, Treas_image.shape[1], Treas_image.shape[0],Treas_image.shape[1]*3, QtGui.QImage.Format_BGR888)
                Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
                self.camera_label.setPixmap(Treas_pixmap)
                self.sublbl.setText("正在进行拍摄识别...")
            camera.release()  # 释放相机资源
            Treas_img_copy = Treas_image.copy()
            Treas_image = reshape_image_scan(Treas_image)[0]
            copy_img = Treas_image

            Treas_image, contours, yellow = Identify.FindColorOne(Treas_img_copy, 1)  # 黄色
            Treas_image, contours, green = Identify.FindColorOne(Treas_img_copy, 2)  # 绿色
            Treas_image, contours, blue = Identify.FindColorOne(Treas_img_copy, 0)  # 蓝色
            Treas_image, contours, red = Identify.FindRedOne(Treas_img_copy, contours)  # 红色
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

            if Treasure != "None":
                Identify.ShapeDetection(copy_img, contours, Treasure)  #形状检测
            Treas_qimg = QtGui.QImage(copy_img.data, copy_img.shape[1], copy_img.shape[0],copy_img.shape[1]*3, QtGui.QImage.Format_BGR888)
            Treas_pixmap = QtGui.QPixmap.fromImage(Treas_qimg)
            self.camera_label.setPixmap(Treas_pixmap)
            # time.sleep(1)

            # TODO 发送撞击指令与否
            if car_color_group == "BLUE":
                if Treasure == "BlueTrue":
                    # serialapi.communicate(0xaa,0xd0,0x00,0x00,0x00,0x00,0x00) # 发送撞击指令
                    # while True:
                    #     if str(serialapi.recv)[0:14] == 'aa01d000000000': break # 等待接收到回复信号
                    Treasure_if_hittable = 1 # 宝藏点可撞击
                    self.sublbl.setText("可以撞击，已注入撞击指令")
                    print("可以撞击，已注入撞击指令")
                else:
                    Treasure_if_hittable = 0 # 宝藏点不可撞击
                    print("不可以撞击，未注入撞击指令")
            elif car_color_group == "RED":
                if Treasure == "RedTrue":
                    # serialapi.communicate(0xaa,0xd0,0x00,0x00,0x00,0x00,0x00)
                    # while True:
                    #     if str(serialapi.recv)[0:14] == 'aa01d000000000': break # 等待接收到回复信号
                    Treasure_if_hittable = 1 # 宝藏点可撞击
                    self.sublbl.setText("可以撞击，已注入撞击指令")
                    print("可以撞击，已注入撞击指令")
                else:
                    Treasure_if_hittable = 0 # 宝藏点不可撞击
                    print("不可以撞击，未注入撞击指令")
            serialapi.recv = str.encode('xxxxxxxxxxx')
            self.camera_label.setPixmap(map_gui_image)
        
        # TODO 到达终点
        self.sublbl.setText('Misson Complete');self.lbl.setText('完成运行,耗时'+'{:.1f}'.format(time.time() - calctimer)+'秒')
        pixmap = QtGui.QPixmap('layla.png')
        # 在 QLabel 控件中显示图片
        self.camera_label.setPixmap(pixmap)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())