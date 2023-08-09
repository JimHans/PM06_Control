#-*-coding:utf-8-*-
# Function: Astar algorithm
#? AStar算法实现模组
#TODO Version 2.1.20230805
#! 依赖项目：numpy networkx itertools
#! 被引用：main.py
import numpy as np
import networkx as nx # 导入网络图模块
import itertools # 导入迭代器模块


#* A*算法地图存储
class Map(object):
    def __init__(self,mapdata,startx,starty,endx,endy):
        self.data = mapdata
        self.startx = startx
        self.starty = starty
        self.endx = endx
        self.endy = endy


#* A*算法Node类
class MapNode(object): 
    #* 初始化节点信息
    def __init__(self,x,y,g,h,father):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.father = father
    #* 处理边界和障碍点 返回可以从当前节点到达的相邻节点列表
    def getNeighbor(self,mapdata,endx,endy):
        x = self.x
        y = self.y
        result = []
    #先判断是否在上下边界
    #上
        if(x!=0 and mapdata[x-1][y]!=0):
            upNode = MapNode(x-1,y,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(upNode)
    #下
        if(x!=len(mapdata)-1 and mapdata[x+1][y]!=0):
            downNode = MapNode(x+1,y,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(downNode)
    #左
        if(y!=0 and mapdata[x][y-1]!=0):
            leftNode = MapNode(x,y-1,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(leftNode)
    #右
        if(y!=len(mapdata[0])-1 and mapdata[x][y+1]!=0):
            rightNode = MapNode(x,y+1,self.g+10,(abs(x-endx)+abs(y-endy))*10,self)
            result.append(rightNode)
        return result
    #* 判断当前节点是否在列表中
    def hasNode(self,worklist):
        for i in worklist:
            if(i.x==self.x and i.y ==self.y):
                return True
        return False
    #* 找到到达该节点的更好路径时更新节点的 g 值
    def changeG(self,worklist):
        for i in worklist:
            if(i.x==self.x and i.y ==self.y):
                if(i.g>self.g):
                    i.g = self.g


# element排序函数，已使用匿名函数改写
# def getKeyforSort(element:MapNode):
#     return element.g #element#不应该+element.h，


#* A*算法主函数 
def astar(workMap):
    startx,starty = workMap.startx,workMap.starty
    endx,endy = workMap.endx,workMap.endy
    startNode = MapNode(startx, starty, 0, 0, None)
    openList = [];lockList = []
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
        # openList.sort(key=getKeyforSort) # 关键步骤
        openList=sorted(openList,key=lambda element:element.g)
        currNode = openList.pop(0)
        lockList.append(currNode)
    result = []
    while(currNode.father!=None):
        result.append((currNode.x,currNode.y))
        currNode = currNode.father
    result.append((currNode.x,currNode.y))
    return result


def tsp(boardmap,treasureinmap):
    '''tsp部分'''
    # 计算起点(18,0)到各个宝藏的距离
    firstdistance = []
    for i in range(len(treasureinmap)):
        map = Map(boardmap, 18, 0, treasureinmap[i][1], treasureinmap[i][0])
        result = astar(map)
        firstdistance.append(len(result) - 1)
    # print("firstdistance:", firstdistance)
    # 将距离起点最近的宝藏放在第一个
    for dis in range(len(firstdistance)):
        if firstdistance[dis] == min(firstdistance):
            treasureinmap[0],treasureinmap[dis] = treasureinmap[dis],treasureinmap[0]
            # temp = treasureinmap[0]
            # treasureinmap[0] = treasureinmap[i]
            # treasureinmap[i] = temp
    # print("treasureinmap1:", treasureinmap)
    # 计算各个宝藏之间的距离
    distances = np.zeros((8, 8)) # [[0 for i in range(8)] for j in range(8)]
    result_matrix = [[0]*8 for _ in range(8)]
    for i in range(8):
        for j in range(8):
            if i < j:
                map = Map(boardmap, treasureinmap[i][1], treasureinmap[i][0], treasureinmap[j][1],
                            treasureinmap[j][0])
                result = astar(map)
                result_matrix[i][j] = result
                result_matrix[j][i] = result
                distances[i][j] = len(result) - 1
                distances[j][i] = len(result) - 1
    # print("距离矩阵distance:", distances)
    # 停止计时
    #print("计算距离矩阵用时：", end - start)
    # 计算最短路径
    # 定义城市和距离矩阵
    cities = [1, 2, 3, 4, 5, 6, 7, 8]
    # 创建完全图
    G = nx.Graph()
    G.add_nodes_from(cities)
    for i, j in itertools.combinations(range(len(cities)), 2):
        G.add_edge(cities[i], cities[j], weight=distances[i][j])

    # 求解旅行商问题
    shortest_tour = None
    min_tour_length = float('inf')
    for permutation in itertools.permutations(cities):
        tour_length = sum([G[permutation[i]][permutation[i + 1]]['weight'] for i in range(len(cities) - 1)])
        tour_length += G[permutation[-1]][permutation[0]]['weight']
        if tour_length < min_tour_length:
            shortest_tour = permutation
            min_tour_length = tour_length
    print("最短路径：", shortest_tour, "总距离成本",min_tour_length)

    order = [0] * 8
    for i in range(8):
        order[i] = shortest_tour[i]-1
    # 将最优线路中的宝藏按顺序进行重新排列
    temp_treasure_map = [0]*8
    for index,orderget in enumerate(order):
        temp_treasure_map[index] = treasureinmap[orderget]
    treasureinmap = temp_treasure_map
    print("最优线路:", treasureinmap)
    return treasureinmap,distances,result_matrix
    '''tsp部分结束'''