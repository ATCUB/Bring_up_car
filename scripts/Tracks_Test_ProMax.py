# coding=UTF-8
import numpy as np
import operator
from itertools import combinations
from itertools import permutations


"定义一个类，存放节点各种属性"
class Treasure_Index:
    "对序号经行处理"

    def __init__(self, Num, NextNums, Distances, point):
        self.Num = Num
        self.NextNums = NextNums
        self.Distance = Distances
        self.point = point
        self.track = []
        self.cost = 0

"对类经行初始化处理"
def Treasure_Indexs_Init():
    # 给序号赋予基础信息
    global Treasure_Indexs
    Treasure_Indexs = [Treasure_Index(0, [1, 16], [1, 2], [0, 0]) for i in range(0, 78)]
    Treasure_Indexs[0] = Treasure_Index(0, [1, 16], [1, 2], [0, 0])
    Treasure_Indexs[1] = Treasure_Index(1, [0, 2, 8], [1, 2, 1], [1, 0])
    Treasure_Indexs[2] = Treasure_Index(2, [1, 3, 10], [2, 1, 1], [3, 0])
    Treasure_Indexs[3] = Treasure_Index(3, [2, 11], [1, 1], [4, 0])
    Treasure_Indexs[4] = Treasure_Index(4, [5, 12], [2, 1], [5, 0])
    Treasure_Indexs[5] = Treasure_Index(5, [4, 22], [2, 2], [7, 0])
    Treasure_Indexs[6] = Treasure_Index(6, [7, 14], [1.5, 1], [8, 0])
    Treasure_Indexs[7] = Treasure_Index(7, [6], [1.5], [9, 0])
    Treasure_Indexs[8] = Treasure_Index(8, [1], [1], [1, 1])
    Treasure_Indexs[9] = Treasure_Index(9, [10, 18], [1, 1], [2, 1])
    Treasure_Indexs[10] = Treasure_Index(10, [9, 2], [1, 1], [3, 1])
    Treasure_Indexs[11] = Treasure_Index(11, [3, 12, 20], [1, 1, 1], [4, 1])
    Treasure_Indexs[12] = Treasure_Index(12, [11, 4, 13], [1, 1, 1], [5, 1])
    Treasure_Indexs[13] = Treasure_Index(13, [13, 21], [1, 1], [6, 1])
    Treasure_Indexs[14] = Treasure_Index(14, [6, 15], [1, 1], [8, 1])
    Treasure_Indexs[15] = Treasure_Index(15, [14, 24], [1, 1], [9, 1])
    Treasure_Indexs[16] = Treasure_Index(16, [0], [2], [0, 2])
    Treasure_Indexs[17] = Treasure_Index(17, [18, 26], [1, 1], [1, 2])
    Treasure_Indexs[18] = Treasure_Index(18, [17, 9, 19, 34], [1, 1, 1, 1], [2, 2])
    Treasure_Indexs[19] = Treasure_Index(19, [18, 27], [1, 1], [3, 2])
    Treasure_Indexs[20] = Treasure_Index(20, [11, 21], [1, 2], [4, 2])
    Treasure_Indexs[21] = Treasure_Index(21, [20, 13, 29], [2, 1, 1], [6, 2])
    Treasure_Indexs[22] = Treasure_Index(22, [5], [2], [7, 2])
    Treasure_Indexs[23] = Treasure_Index(23, [24, 31], [1, 1], [8, 2])
    Treasure_Indexs[24] = Treasure_Index(24, [23, 15], [1, 1], [9, 2])
    Treasure_Indexs[25] = Treasure_Index(25, [26, 39], [1, 2], [0, 3])
    Treasure_Indexs[26] = Treasure_Index(26, [25, 17], [1, 1], [1, 3])
    Treasure_Indexs[27] = Treasure_Index(27, [19, 28, 35], [1, 2, 1], [3, 3])
    Treasure_Indexs[28] = Treasure_Index(28, [27], [2], [5, 3])
    Treasure_Indexs[29] = Treasure_Index(29, [21, 30], [1, 1], [6, 3])
    Treasure_Indexs[30] = Treasure_Index(30, [29, 31, 37], [1, 1, 1], [7, 3])
    Treasure_Indexs[31] = Treasure_Index(31, [30, 23], [1, 1], [8, 3])
    Treasure_Indexs[32] = Treasure_Index(32, [38], [1], [9, 3])
    Treasure_Indexs[33] = Treasure_Index(33, [34], [1], [1, 4])
    Treasure_Indexs[34] = Treasure_Index(34, [33, 18], [1, 2], [2, 4])
    Treasure_Indexs[35] = Treasure_Index(35, [27, 36, 41], [1, 3, 1], [3, 4])
    Treasure_Indexs[36] = Treasure_Index(36, [35, 37, 42], [3, 1, 1], [6, 4])
    Treasure_Indexs[37] = Treasure_Index(37, [36, 30, 38], [1, 1, 2], [7, 4])
    Treasure_Indexs[38] = Treasure_Index(38, [37, 32, 52], [2, 1, 1], [9, 4])
    Treasure_Indexs[39] = Treasure_Index(39, [25, 40, 45], [2, 2, 1], [0, 5])
    Treasure_Indexs[40] = Treasure_Index(40, [39, 41, 47], [2, 1, 1], [2, 5])
    Treasure_Indexs[41] = Treasure_Index(41, [40, 35, 42], [1, 1, 2], [3, 5])
    Treasure_Indexs[42] = Treasure_Index(42, [41, 36, 50], [3, 1, 1], [6, 5])
    Treasure_Indexs[43] = Treasure_Index(43, [44, 59], [1, 2], [7, 5])
    Treasure_Indexs[44] = Treasure_Index(44, [43], [1], [8, 5])
    Treasure_Indexs[45] = Treasure_Index(45, [39], [1], [0, 6])
    Treasure_Indexs[46] = Treasure_Index(46, [47, 54], [1, 1], [1, 6])
    Treasure_Indexs[47] = Treasure_Index(47, [46, 40, 48], [1, 1, 1], [2, 6])
    Treasure_Indexs[48] = Treasure_Index(48, [47, 56], [1, 1], [3, 6])
    Treasure_Indexs[49] = Treasure_Index(49, [50], [2], [4, 6])
    Treasure_Indexs[50] = Treasure_Index(50, [49, 42, 58], [2, 1, 1], [6, 6])
    Treasure_Indexs[51] = Treasure_Index(51, [52, 60], [1, 1], [8, 6])
    Treasure_Indexs[52] = Treasure_Index(52, [51, 38], [1, 2], [9, 6])
    Treasure_Indexs[53] = Treasure_Index(53, [54, 62], [1, 1], [0, 7])
    Treasure_Indexs[54] = Treasure_Index(54, [53, 46], [1, 1], [1, 7])
    Treasure_Indexs[55] = Treasure_Index(55, [72], [2], [2, 7])
    Treasure_Indexs[56] = Treasure_Index(56, [48, 57, 64], [1, 2, 1], [3, 7])
    Treasure_Indexs[57] = Treasure_Index(57, [56, 66], [2, 1], [5, 7])
    Treasure_Indexs[58] = Treasure_Index(58, [50, 59], [1, 1], [6, 7])
    Treasure_Indexs[59] = Treasure_Index(59, [58, 43, 60, 68], [1, 2, 1, 1], [7, 7])
    Treasure_Indexs[60] = Treasure_Index(60, [59, 51], [1, 1], [8, 7])
    Treasure_Indexs[61] = Treasure_Index(61, [77], [2], [9, 7])
    Treasure_Indexs[62] = Treasure_Index(62, [53, 63], [1, 1], [0, 8])
    Treasure_Indexs[63] = Treasure_Index(63, [62, 71], [1, 1], [1, 8])
    Treasure_Indexs[64] = Treasure_Index(64, [56, 65], [1, 1], [3, 8])
    Treasure_Indexs[65] = Treasure_Index(65, [64, 66, 73], [1, 1, 1], [4, 8])
    Treasure_Indexs[66] = Treasure_Index(66, [65, 57, 74], [1, 1, 1], [5, 8])
    Treasure_Indexs[67] = Treasure_Index(67, [68, 75], [1, 1], [6, 8])
    Treasure_Indexs[68] = Treasure_Index(68, [67, 59], [1, 1], [7, 8])
    Treasure_Indexs[69] = Treasure_Index(69, [76], [1], [8, 8])
    Treasure_Indexs[70] = Treasure_Index(70, [71], [1], [0, 9])
    Treasure_Indexs[71] = Treasure_Index(71, [70, 63], [2, 1], [1, 9])
    Treasure_Indexs[72] = Treasure_Index(72, [55, 73], [2, 2], [2, 9])
    Treasure_Indexs[73] = Treasure_Index(73, [72, 65], [2, 1], [4, 9])
    Treasure_Indexs[74] = Treasure_Index(74, [66, 75], [1, 1], [5, 9])
    Treasure_Indexs[75] = Treasure_Index(75, [74, 67, 76], [1, 1, 2], [6, 9])
    Treasure_Indexs[76] = Treasure_Index(76, [75, 69, 77], [2, 1, 1], [8, 9])
    Treasure_Indexs[77] = Treasure_Index(77, [76, 61], [1, 2], [9, 9])
    return Treasure_Indexs

"寻找外围点中代价最小的点"
def FindMinOutPoint(OutPoints):
    Cost = []
    for OutPoint in OutPoints:
        Cost.append(Treasure_Indexs[OutPoint].cost)
    min_index, min_number = min(enumerate(Cost), key=operator.itemgetter(1))
    return OutPoints[min_index]

"判断当前节点可否通行"
def GoOrNot(goal_point):
    if len(Treasure_Indexs[goal_point].NextNums) > 0:
        return True
    else:
        return False

"判断当前点可通行的点个数"
def PaneelNum(goal_point):
    Num = len(Treasure_Indexs[goal_point].NextNums)
    return Num

"防止线路重合"
def AddTrack(point, point_tracks):
    flag = 1;
    for point_track in point_tracks:
        if point == point_track:
            flag = 0
    if flag == 1:
        point_tracks.append(point)
    return point_tracks

"将路径节点转换为坐标点"
def GetPoints(Tracks):
    # Treasure_Indexs_Init()
    points = [300,300]
    i = 0
    # if type(Tracks) == int :
    # points.append(Treasure_Indexs[Tracks].point)
    # points = ((Treasure_Indexs[Tracks].point[0] * 40) + 20,(Treasure_Indexs[Tracks].point[1] * 40) + 20)
    points[0] = (Treasure_Indexs[Tracks].point[0] * 40) + 20
    points[1] = (Treasure_Indexs[Tracks].point[1] * 40) + 20
    # else:
    #     for Track in Tracks:
    #         points.append(Treasure_Indexs[Track].point)
    #         points[i][0] = (Treasure_Indexs[Track].point[0] * 40) + 20
    #         points[i][1] = (Treasure_Indexs[Track].point[1] * 40) + 20
    #         i = i + 1
    return points

"根据坐标返回节点号"
def LocationChange(x,y):
    for Treasure_Index in Treasure_Indexs:
        if x == Treasure_Index.point[0]:
            if y == Treasure_Index.point[1]:
                return Treasure_Index.Num

"根据坐标返回当前节点号"
def GetLocation(x,y):
    x = x // 40
    y = y // 40
    for Treasure_Index in Treasure_Indexs:
        num = Treasure_Index.Num
        test_x = Treasure_Index.point[0]
        if x == Treasure_Index.point[0]:
            test_y = Treasure_Index.point[1]
            if y == Treasure_Index.point[1]:
                return Treasure_Index.Num



"根据当前位置，判断下一个位置该怎么移动"
def GetDirection(Tracks,points):
    i = 0
    for Track in Tracks :
        if Track == points :
            if Treasure_Indexs[Track].point[0] < Treasure_Indexs[Tracks[i+1]].point[0]:
                return 180
            if Treasure_Indexs[Track].point[0] > Treasure_Indexs[Tracks[i + 1]].point[0]:
                return 0
            if Treasure_Indexs[Track].point[1] > Treasure_Indexs[Tracks[i + 1]].point[1]:
                return 90
            if Treasure_Indexs[Track].point[1] < Treasure_Indexs[Tracks[i + 1]].point[1]:
                return 270
        i = i + 1
    return "false"


"获取下一个路口的路况"
# 1是L
# 2是T，向上看
# 3是T，向左右看
# 4是十字

def GetNextRoad(Location1_X,Location1_Y,Location2_X,Location2_Y):
    Location1 = GetLocation(Location1_X, Location1_Y)
    Location1_X = Treasure_Indexs[Location1].point[0]
    Location1_Y = Treasure_Indexs[Location1].point[1]
    # print(Location1)
    Location2 = GetLocation(Location2_X, Location2_Y)
    # print(Location2)
    if(len(Treasure_Indexs[Location2].NextNums) == 2 ) :
        return 1
    if(len(Treasure_Indexs[Location2].NextNums) == 3 ) :
        NextNums = Treasure_Indexs[Location2].NextNums
        NextPoints = []
        for NextNum in NextNums:
            NextPoints.append((Treasure_Indexs[NextNum].point[0],Treasure_Indexs[NextNum].point[1]))
        print(NextPoints)
        for NextPoint in NextPoints :
            if (Location1_X == NextPoint[0]) & (Location1_Y == NextPoint[1]):
                NextPoints.remove(NextPoint)
        print(NextPoints)
        dirction_R = 0
        dirction_L = 0
        dirction_U = 0
        dirction_D = 0
        for NextPoint in NextPoints:
            if (Treasure_Indexs[Location2].point[0] - NextPoint[0] >0):
                dirction_L = dirction_L + 1
            if (Treasure_Indexs[Location2].point[0] - NextPoint[0] <0):
                dirction_R = dirction_R + 1
            if (Treasure_Indexs[Location2].point[1] - NextPoint[1] <0):
                dirction_D = dirction_D + 1
            if (Treasure_Indexs[Location2].point[1] - NextPoint[1] >0):
                dirction_U = dirction_U + 1
        print(dirction_R)
        print(dirction_L)
        print(dirction_U)
        print(dirction_D)
        if ((dirction_R == 1)&(dirction_L == 1))or((dirction_D == 1)&(dirction_U == 1)):
            return 2
        else:
            return 3
    if (len(Treasure_Indexs[Location2].NextNums) == 4):
        return 4



"获取索引的路径"
def GetTrack(H):
    if(H==1):
        return Global_Track_1
    if (H == 2):
        return Global_Track_2
    if (H == 3):
        return Global_Track_3
    if (H == 4):
        return Global_Track_4
    if (H == 5):
        return Global_Track_5
    if (H == 6):
        return Global_Track_6
    if (H == 7):
        return Global_Track_7
    if (H == 8):
        return Global_Track_8
    if (H == 9):
        return Global_Track_9

# 输入：
# Location_X坐标X
# Location_Y坐标Y
# H 第几条路径
# 返回：
# 0是-|或者|-这种道路直走
# 1是T左转，
# 2是T右转，
# 1是-|左转，
# 3是走L，
# 2是|-右转
# 0是十字直行
# 2是十字右转
# 1是十字左转
"寻找当前坐标的下一个路口怎么走"
def GetNextDirction(Location_X,Location_Y,H):
    Location = GetLocation(Location_X,Location_Y)
    num = 0
    Indexs = GetTrack(H)
    # print("Index=",Indexs)
    for Index in Indexs:
        if Location == Index:
            break
        num = num + 1

    # print("num = ",num)
    if (num < len(Indexs) - 1) & (num > 0):
        point1 = Treasure_Indexs[Indexs[num - 1]].point
        point2 = Treasure_Indexs[Indexs[num]].point
        point3 = Treasure_Indexs[Indexs[num + 1]].point
        vector1 = (point2[0] - point1[0], point2[1] - point1[1])
        vector2 = (point3[0] - point2[0], point3[1] - point2[1])
        vector = (vector2[0] - vector1[0], vector2[1] - vector1[1])
        print("point1=", Treasure_Indexs[Indexs[num - 1]].Num)
        print("point2=", Treasure_Indexs[Indexs[num]].Num)
        print("point3=", Treasure_Indexs[Indexs[num + 1]].Num)
        print("vector1=", vector1)
        print("vector2=", vector2)
        print("vector=", vector)
        dirction = 0
        if(vector1[0]>0):
            if(vector2[1]>0):
                dirction = 1
            if (vector2[1] < 0):
                dirction = 3
        elif (vector1[0] < 0):
            if (vector2[1] > 0):
                dirction = 3
            if (vector2[1] < 0):
                dirction = 1
        elif(vector1[1]>0):
            if (vector2[0] > 0):
                dirction = 3
            if (vector2[0] < 0):
                dirction = 1
        elif (vector1[1] < 0):
            if (vector2[0] > 0):
                dirction = 1
            if (vector2[0] < 0):
                dirction = 3
        else:
            dirction = 0
        print("dirction=", dirction)
        Road = GetNextRoad(Treasure_Indexs[Indexs[num - 1]].point[0] * 40 + 20,
                           Treasure_Indexs[Indexs[num - 1]].point[1] * 40 + 20,
                           Treasure_Indexs[Indexs[num]].point[0] * 40 + 20,
                           Treasure_Indexs[Indexs[num]].point[1] * 40 + 20)
        print("road = ", Road)
        if (Road == 1):
            print("走L")
            if (dirction == 3):
                print("L左转")
                return 1
            if (dirction == 1):
                print("L右转")
                return 2
            # return 3

        if (Road == 2):
            if (dirction == 3):
                print("T左转")
                return 1
            if (dirction == 1):
                print("T右转")
                return 2
        if (Road == 3):
            if (dirction == 0):
                print("-|或者|-这种道路直走")
                return 0
            if (dirction == 1):
                print("-|右转")
                return 2
            if (dirction == 3):
                print("-|左转")
                return 1
        if (Road ==4):
            if (dirction == 0):
                print("十字道路直走")
                return 0
            if (dirction == 1):
                print("十字右转")
                return 2
            if (dirction == 3):
                print("十字左转")
                return 1

    else:
        print("直走")
        # returnz

# H是第几条路径
"返回路径的所有方向点"
def GetNextDirctions(H):
    Tracks = GetTrack(H)
    # print(Tracks)
    Dir = []
    for Track in Tracks:
        point = GetPoints(Track)
        # print("print(point[0][0])",point[0])
        # print("print(point[0][1])",point[1])
        x = point[0]
        y = point[1]
        # print(GetNextDirction(x, y, H))
        if (GetNextDirction(x,y,H)!= None):
            Dir.append(GetNextDirction(x,y,H))

    return Dir

"定义路径寻找函数，作为主要函数"
def FindTrack(first_point, last_point):
    global Track_Cost
    "目标点从初始点开始"
    Treasure_Indexs_Init()
    goal_point = first_point
    "设置已经经过的点"
    point_tracks = []
    "设置外围点，为外围点添加第一个量"
    OutPoint = []
    OutPoint.append(first_point)
    "为第一个节点添加属性"
    
    Treasure_Indexs[goal_point].track.append(goal_point)
    point_tracks.append(goal_point)
    "当目标点不为终点时"
    while Treasure_Indexs[goal_point].Num != Treasure_Indexs[last_point].Num:
        "找出最外围点中代价最小的点"
        goal_point = FindMinOutPoint(OutPoint)
        "判断这个点是否有可通行点"
        if GoOrNot(goal_point) == True:
            i = 0
            "删除来时路径"
            for NextNum in Treasure_Indexs[goal_point].NextNums:
                if NextNum == Treasure_Indexs[goal_point].track[len(Treasure_Indexs[goal_point].track) - 2]:
                    del Treasure_Indexs[goal_point].NextNums[i]
                    del Treasure_Indexs[goal_point].Distance[i]
                i = i + 1
            i = 0
            "删除重复路径"
            for NextNum in Treasure_Indexs[goal_point].NextNums:
                for point_track in point_tracks:
                    if NextNum == point_track:
                        del Treasure_Indexs[goal_point].NextNums[i]
                        del Treasure_Indexs[goal_point].Distance[i]
                i = i + 1

            "判断有几个可通行点"
            "无可通行点"
            if PaneelNum(goal_point) == 0:
                Treasure_Indexs[goal_point].cost = 999
                OutPoint.remove(goal_point)
            "一个可通行点"
            if PaneelNum(goal_point) == 1:
                "目标点索引"
                goal_point1 = goal_point
                goal_point = Treasure_Indexs[goal_point].NextNums[0]
                "添加目标点路径"
                Treasure_Indexs[goal_point].track = Treasure_Indexs[goal_point1].track
                Treasure_Indexs[goal_point].track.append(goal_point)
                AddTrack(goal_point, point_tracks)
                track = Treasure_Indexs[goal_point].track
                "添加目标点代价值"
                Treasure_Indexs[goal_point].cost = Treasure_Indexs[goal_point1].cost + \
                                                   Treasure_Indexs[goal_point1].Distance[0]
                cost = Treasure_Indexs[goal_point].cost
                "删除原有点，添加外围点"
                OutPoint.remove(goal_point1)
                OutPoint.append(goal_point)
                if goal_point == last_point:
                    # print(Treasure_Indexs[goal_point].track)
                    break

                "两个个可通行点"
            elif PaneelNum(goal_point) >= 2:
                "目标点索引"
                i = 0
                goal_point1 = goal_point
                for NextNum in Treasure_Indexs[goal_point].NextNums:
                    "目标点索引"
                    goal_point = NextNum
                    "添加目标点路径"
                    Treasure_Indexs[goal_point].track = Treasure_Indexs[goal_point1].track[:]
                    Treasure_Indexs[goal_point].track.append(goal_point)
                    AddTrack(goal_point, point_tracks)
                    track = Treasure_Indexs[goal_point].track
                    "添加目标点代价值"
                    Treasure_Indexs[goal_point].cost = Treasure_Indexs[goal_point1].cost + \
                                                       Treasure_Indexs[goal_point1].Distance[i]
                    cost = Treasure_Indexs[goal_point].cost
                    "删除原有点，添加外围点"
                    if i == 0:
                        OutPoint.remove(goal_point1)
                    OutPoint.append(goal_point)
                    i = i + 1
                    if goal_point == last_point:
                        # print(Treasure_Indexs[goal_point].track)
                        break

    tracks = Treasure_Indexs[goal_point].track
    Track_Cost.append(Treasure_Indexs[goal_point].cost)
    for i in range(0, len(tracks) - 1):
        x = Treasure_Indexs[tracks[i + 1]].point[0] - Treasure_Indexs[tracks[i]].point[0]
        y = Treasure_Indexs[tracks[i + 1]].point[1] - Treasure_Indexs[tracks[i]].point[1]
        if x > 0:
            1
            # print("向右")
        if x < 0:
            1
            # print("向左")
        if y > 0:
            1
            # print("向下")
        if y < 0:
            1
            # print("向上")
    return Treasure_Indexs[last_point].track

# Treaseures是所有宝藏的位置，无返回值
"用于文件内部确定所有路径"
def FindTracks(Treaseures):
    global Global_Track_1
    global Global_Track_2
    global Global_Track_3
    global Global_Track_4
    global Global_Track_5
    global Global_Track_6
    global Global_Track_7
    global Global_Track_8
    global Global_Track_9
    Treasures_Index = []
    Global_Track_1 = []
    Global_Track_2 = []
    Global_Track_3 = []
    Global_Track_4 = []
    Global_Track_5 = []
    Global_Track_6 = []
    Global_Track_7 = []
    Global_Track_8 = []
    Global_Track_9 = []
    # print("len(Treaseures)",len(Treaseures))
    for Treasure in Treaseures:
        Treasures_Index.append(GetLocation(Treasure[0],Treasure[1]))
    if (len(Treaseures) == 2):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 3):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 4):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 5):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
        Global_Track_4 = FindTrack(Treasures_Index[3], Treasures_Index[4])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 6):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
        Global_Track_4 = FindTrack(Treasures_Index[3], Treasures_Index[4])
        Treasure_Indexs_Init()
        Global_Track_5 = FindTrack(Treasures_Index[4], Treasures_Index[5])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 7):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
        Global_Track_4 = FindTrack(Treasures_Index[3], Treasures_Index[4])
        Treasure_Indexs_Init()
        Global_Track_5 = FindTrack(Treasures_Index[4], Treasures_Index[5])
        Treasure_Indexs_Init()
        Global_Track_6 = FindTrack(Treasures_Index[5], Treasures_Index[6])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 8):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
        Global_Track_4 = FindTrack(Treasures_Index[3], Treasures_Index[4])
        Treasure_Indexs_Init()
        Global_Track_5 = FindTrack(Treasures_Index[4], Treasures_Index[5])
        Treasure_Indexs_Init()
        Global_Track_6 = FindTrack(Treasures_Index[5], Treasures_Index[6])
        Treasure_Indexs_Init()
        Global_Track_7 = FindTrack(Treasures_Index[6], Treasures_Index[7])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 9):
        Global_Track_1 = FindTrack(Treasures_Index[0], Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
        Global_Track_4 = FindTrack(Treasures_Index[3], Treasures_Index[4])
        Treasure_Indexs_Init()
        Global_Track_5 = FindTrack(Treasures_Index[4], Treasures_Index[5])
        Treasure_Indexs_Init()
        Global_Track_6 = FindTrack(Treasures_Index[5], Treasures_Index[6])
        Treasure_Indexs_Init()
        Global_Track_7 = FindTrack(Treasures_Index[6], Treasures_Index[7])
        Treasure_Indexs_Init()
        Global_Track_8 = FindTrack(Treasures_Index[7], Treasures_Index[8])
        Treasure_Indexs_Init()
    if (len(Treaseures) == 10):
        Global_Track_1 = FindTrack(Treasures_Index[0],Treasures_Index[1])
        Treasure_Indexs_Init()
        Global_Track_2 = FindTrack(Treasures_Index[1], Treasures_Index[2])
        Treasure_Indexs_Init()
        Global_Track_3 = FindTrack(Treasures_Index[2], Treasures_Index[3])
        Treasure_Indexs_Init()
        Global_Track_4 = FindTrack(Treasures_Index[3], Treasures_Index[4])
        Treasure_Indexs_Init()
        Global_Track_5 = FindTrack(Treasures_Index[4], Treasures_Index[5])
        Treasure_Indexs_Init()
        Global_Track_6 = FindTrack(Treasures_Index[5], Treasures_Index[6])
        Treasure_Indexs_Init()
        Global_Track_7 = FindTrack(Treasures_Index[6], Treasures_Index[7])
        Treasure_Indexs_Init()
        Global_Track_8 = FindTrack(Treasures_Index[7], Treasures_Index[8])
        Treasure_Indexs_Init()
        Global_Track_9 = FindTrack(Treasures_Index[8], Treasures_Index[9])
        Treasure_Indexs_Init()



# 传入若干个点的坐标即可完成寻找最短路径
def Get_Min_Tracks(points):
    global  Track_Cost
    global New_Points
    Track_Cost = []
    New_Points = []
    Cost = []
    s1 = []
    s2 = []
    point = []
    Cost_Min = 999
    Cost_Min_Index = [(0,0,0,0),(0,0,0,0)]
    if (len(points)-2 <= 4):
        for i in range(1,len(points)-1):
            s1.append(i)
        points1 = list(permutations(s1, len(points)-2))
        np.array(points1)
        for point1 in points1:
            point = []
            for i in range(0, len(point1)):
                point.append(points[point1[i]])
            Track_Cost = []
            tracks = FindTracks(([points[0]]+point+[points[len(points)-1]]))
            # print(Track_Cost)
            Track = 0
            for track in Track_Cost:
                Track = Track + track
            if (Track < Cost_Min):
                Cost_Min = Track
                Cost_Min_Index[0] = point1
            Cost.append(Track)
            # print(Track)



    if (len(points)-2 >4):
        for i in range(5, len(points)-1):
            s2.append(i)
        s1 = [1, 2, 3, 4]
        points1 = list(permutations(s1, 4))
        points2 = list(permutations(s2, len(points)-6))
        np.array(points1)
        np.array(points2)
        for point1 in points1:
            for point2 in points2:
                point = []
                for i in range(0, len(point1)):
                    point.append(points[point1[i]])
                for i in range(0, len(point2)):
                    point.append(points[point2[i]])
                Track_Cost = []
                tracks = FindTracks(([points[0]]+point+[points[len(points)-1]]))
                Track = 0
                for track in Track_Cost:
                    Track = Track + track
                # print(Track)
                if (Track < Cost_Min):
                    Cost_Min = Track
                    Cost_Min_Index[0] = point1
                    Cost_Min_Index[1] = point2
                Cost.append(Track)
    # print(Cost_Min_Index)
    print("Cost_min is",Cost_Min)
    New_Points.append(points[0])
    for i in range(0, len(Cost_Min_Index[0])):
        if (Cost_Min_Index[0][i] != 0):
            New_Points.append(points[Cost_Min_Index[0][i]])
    if (len(Cost_Min_Index[1])<4):
        for i in range (0,len(Cost_Min_Index[1])):
            New_Points.append(points[Cost_Min_Index[1][i]])
    else:
        for i in range (0,4):
            if (Cost_Min_Index[1][i] != 0):
                New_Points.append(points[Cost_Min_Index[1][i]])

    New_Points.append(points[len(points)-1])
    return New_Points

"坐标点初始化，传入图片处理完后得到的数据即可"
#使用的时候直接读取文件即可# b = np.loadtxt("MinTracksPoints.txt")  # 读取文件
def Point_Init(points):
    new_point = Get_Min_Tracks(points)
    np.savetxt("MinTracksPoints.txt", New_Points)  # 保存文件
    FindTracks(new_point)
    return new_point



"在前四个点检测完之后，进行新的路径规划"
#point为前四个点的状态，1为真，0为假
#如（1，0，1，0）第一个真第二个假，第三个真，第四个假，传入为数组非元组
#返回值为新路径
def ReGetMinTracks(points):
    New_Points = np.loadtxt("MinTracksPoints.txt")  # 读取文件
    New_New_Points = list(New_Points[5:].copy())
    # print("New_New_Points = ",New_New_Points)
    if (points == [1,1,1]):
        New_New_Points = list(New_Points[3:].copy())
        New_New_Points.pop(1)
        New_New_Points.pop(1)
        New_New_Points.pop(1)
        New_New_Points.pop(1)
        New_New_Points.pop(1)
        FindTracks(New_New_Points)
        return New_New_Points
    if(points == [1,1,1,0]):
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
    if (points == [1, 1, 0, 1]):
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
    if (points == [1, 0, 1, 1]):
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
    if (points == [0, 1, 1, 1]):
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)
        New_New_Points.pop(0)

    if (points == [1, 1, 0, 0]):
        New_New_Points.pop(2)
        New_New_Points.pop(2)
    if (points == [1, 0, 1, 0]):
        New_New_Points.pop(1)
        New_New_Points.pop(2)
    if (points == [1, 0, 0, 1]):
        New_New_Points.pop(0)
        New_New_Points.pop(2)
    if (points == [0, 1, 1, 0]):
        New_New_Points.pop(1)
        New_New_Points.pop(1)
    if (points == [0, 1, 0, 1]):
        New_New_Points.pop(0)
        New_New_Points.pop(1)
    if (points == [0, 0, 1, 1]):
        New_New_Points.pop(0)
        New_New_Points.pop(0)

    if (points == [1, 0, 0, 0]):
        New_New_Points.pop(3)
    if (points == [0, 1, 0, 0]):
        New_New_Points.pop(2)
    if (points == [0, 0, 1, 0]):
        New_New_Points.pop(1)
    if (points == [0, 0, 0, 1]):
        New_New_Points.pop(0)

    if (points == [0, 0, 0, 0]):
        None

    New_New_Points.insert(0,New_Points[4])
    FindTracks(New_New_Points)
    # print(np.array(New_New_Points))
    return New_New_Points


"恢复程序，当发送碰撞时"
#传入下一个未到达点坐标即可
def TracksCorrection(point):
    MinTracks = np.loadtxt("MinTracksPoints.txt") # 读取文件
    for MinTrack in MinTracks:
        if(MinTrack[0] == point[0])&(MinTrack[1] == point[1]):
            break
        else:
            MinTracks = np.delete(MinTracks,1,0)
    MinTracks = np.insert(MinTracks, 1, point, 0)
    np.savetxt("CorrectionTracksPoints.txt", MinTracks)  # 保存文件
    print(MinTracks)
    FindTracks(MinTracks)
    return MinTracks




# 给序号赋予基础信息
Track_Cost = []
Treasure_Indexs = [Treasure_Index(0, [1, 16], [1, 2], [0, 0]) for i in range(0, 78)]
Treasure_Indexs_Init()
points = [(20,380),(20,260),(340,220),(380,300),(100,300),(60, 180.),( 20, 100.),(300, 100.),(380,140.),(380,20)]
Point_Init(points)
# print(Track_Cost)
# Point_Init(points)
print(ReGetMinTracks([1,1,0,0]))
# print(np.loadtxt("MinTracksPoints.txt") )
# Dir = GetNextDirctions(1)
# print(Dir)

