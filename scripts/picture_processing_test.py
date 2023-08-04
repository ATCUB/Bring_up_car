# -*- coding:utf-8 -*-
import cv2
import numpy as np
import operator
from Tracks_Test_ProMax_Pro import Point_Init
"转换成左上角坐标"
def Left_Down_LocationChange(Location_x,Location_y):
    Location_x = Location_x
    Location_y = 400 - Location_y
    return Location_x,Location_y


"对图像进行翻转"
def rotate_bound(image, angle):
    # grab the dimensions of the image and then determine the
    # center
    (h, w) = image.shape[:2]
    (cX, cY) = (w / 2, h / 2)


    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., the rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))

    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY

    # perform the actual rotation and return the image
    return cv2.warpAffine(image, M, (nW, nH))

"定义形状检测函数"
def ShapeDetection(img,imge):
    "寻找轮廓点"
    contours = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[1]  #
    # cv2.drawContours(imge, contours, -1, (0, 0, 255), 3)
    # cv2.imshow("imge",imge)
    i = 0
    locate = np.zeros((4,4))
    location = np.zeros((4, 2))
    for obj in contours:
        "计算轮廓内区域的面积"
        area = cv2.contourArea(obj)
        "绘制轮廓线"
        cv2.drawContours(imgContour, obj, -1, (255, 0, 0), 4)
        "计算轮廓周长"
        perimeter = cv2.arcLength(obj,True)
        "获取轮廓角点坐标"
        approx = cv2.approxPolyDP(obj,0.02*perimeter,True)
        "轮廓角点的数量"
        CornerNum = len(approx)
        "当轮廓等于4时，继续运行"
        if CornerNum == 4 :
            if area > 10000 :
                i
            elif area > 800 :
                # print(area)
                x, y, w, h = cv2.boundingRect(approx)  #获取坐标值和宽度、高度
                locate[i] = ( x, y , w , h )
                location[i] = ( x , y )
                # print("locate[i] = ",locate[i])
                #轮廓对象分类
                cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,0,255),2)  #绘制边界框
                i = i + 1
    "当轮廓大于4时，报错弹出"
    if i < 4 :
        print("警告：未检测出 4 个角点,检测出",i,"个点")
        locate = []
    "当轮廓小于4时，报错弹出"
    if i > 4 :
        print("警告：检测过多角点,检测出",i,"个点")
        location = []
    return locate , location

"定义画圆函数"
def drawcircles(img_K,img_K_copy):
    if 1:
        # 读入地图图片
        map_image = img_K

        # 转换为灰度图
        gray_map = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)

        # 图像二值化
        ret, thresh_map = cv2.threshold(gray_map, 127, 255, cv2.THRESH_BINARY)

        for x in range(0,50) :
            for y in range(0,50):
                thresh_map[x,y] = 255
        # cv2.imshow("thresh_ma1p", thresh_map)
        # imge = thresh_map
        circles = cv2.HoughCircles(thresh_map, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=16, minRadius=1,
                                   maxRadius=18)
        if circles is  None:
            print("未检测到圆")
            return 0 , 999
        circless = circles[0, :]
        # print("len(circless)", len(circless))
        canshu = 16
        i = 0
        while len(circless) != 8:
            if len(circless) < 8:
                canshu = canshu - 1
                circles = cv2.HoughCircles(thresh_map, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=canshu,
                                           minRadius=1,
                                           maxRadius=18)
                circless = circles[0, :]

            else:
                canshu = canshu + 1
                circles = cv2.HoughCircles(thresh_map, cv2.HOUGH_GRADIENT, 1, 20, param1=100, param2=canshu,
                                           minRadius=1,
                                           maxRadius=18)
                circless = circles[0, :]
            i = i + 1
            if i > 100 :
                print("未检测到8个圆")
                flag = 999
                return 0 ,999
        print("len(circless)", len(circless))
        i= 0
        # print("圆心坐标：", circles)
        # print("圆心坐标：", circles.shape)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                if x < 50 :
                    index_circle = i
                    circles = np.delete(circles, index_circle ,0)
                    # print("圆心坐标：", circles.shape)
                    # print("index_circle",index_circle)
                else:
                    cv2.circle(img_K_copy, (x, y), r, (0, 255, 0), 6)
                i = i + 1
                # cv2.circle(thresh_map, (x, y), r, (255, 255, 255), 16)
                # cv2.rectangle(img_K, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            # cv2.imshow("draw circles 0 ", img_K_copy)
            circles_x = circles[:, 0]
            # circles_x = np.delete(circles_x, i)
            circles_y = circles[:, 1]
            # circles_y = np.delete(circles_y, i)
            treasure = np.zeros((len(circles), 2))
            treasure_x = np.zeros((len(circles),))
            treasure_y = np.zeros((len(circles),))
            # print("circles_x：", circles_x)
            # print("circles_y：", circles_y)
            i = 0
            for circle in circles:
                treasure_x[i] = (circles_x[i] - mistake) // 40
                treasure_y[i] = (circles_y[i] - mistake) // 40
                treasure[i] = (treasure_x[i] * 40 + 20, treasure_y[i] * 40 + 20)
                i = i + 1

            # print("treasure_x = ", treasure_x)
            # print("treasure_y = ", treasure_y)
            # print("treasure = ", treasure)
            flag = 1
            return treasure ,flag
        else:
            flag = 0
            treasure = 0
            return treasure,flag

"将宝藏按照人为顺序重新排列"
def ReTreasure(treasures,flag):
    if flag == 0 :
        treasuresss = np.zeros((8, 2))
        treasuress = []
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x < 80) & (y > 200) :
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x > 160) & (y > 200) & (y < 280):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x > 240) & (y > 280):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if ((x > 80) & (x < 160) & (y > 200)) | ((x > 160) & (x < 240) & (y > 280)):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x < 240) & (y > 120) & (y < 200) :
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x < 160) & (y < 120):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if ((x > 160) & (x < 320) & (y < 120)) | ((x > 240) & (x < 320) & (y < 200)):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x > 320) & (y < 200):
                treasuress.append(treasure)
        i = 0
        # print("treasuress=", treasuress)
        for treasure in treasuress :
            # print("treasuress=", treasuress[i])
            treasuresss[i] = treasure
            i = i + 1
        return treasuresss
    if flag == 1 :
        treasuresss = np.zeros((8, 2))
        treasuress = []
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x > 320) & (y < 200):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x < 240) & (y > 120) & (y < 200) :
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x < 160) & (y < 120):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if ((x > 160) & (x < 320) & (y < 120)) | ((x > 240) & (x < 320) & (y < 200)):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x > 160) & (y > 200) & (y < 280):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x > 240) & (y > 280):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if ((x > 80) & (x < 120) & (y > 200)) | ((x > 160) & (x < 240) & (y > 280)):
                treasuress.append(treasure)
        for treasure in treasures:
            x = treasure[0]
            y = treasure[1]
            if (x < 80) & (y > 200) :
                treasuress.append(treasure)
        i = 0

        for treasure in treasuress:
            # print("treasuress=", treasuress[i])
            treasuresss[i] = treasure
            i = i + 1

        return treasuresss




def Find_Treasure(color):
    "获取照片"
    cap = cv2.VideoCapture(0)  # 0表示默认相机设备
    Treasures_Check = []
    if not cap.isOpened():
        print("Failed to open camera")
        exit()
    while(True):
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            exit()
    #对藏宝图参数进行设置
        blue = 0
        red = 1
        # color  = blue
        global  mistake
        mistake = 83
        #读入图片并修正大小
        # img = cv2.imread("../photo/test14.png")
        img = frame
        # print("img.shape",img.shape)

        "对图像进行方向修改"
        if img.shape[0] > img.shape[1] :
            img_xz = rotate_bound(img,-90)
            img = img_xz

        "对图像进行大小修改"
        ratio = img.shape[0] / 500.0
        rate = img.shape[1] / img.shape[0]
        Len = rate*500.0
        orig = img.copy()
        img = cv2.resize(img,(int(Len),500))
        cv2.imshow("map", img)
        global imgContour
        imgContour = img.copy()
        # cv2.imshow("img",img)

        #对图像进行初步处理
        "转灰度图"
        imgGray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        "高斯模糊"
        imgBlur = cv2.GaussianBlur(imgGray,(5,5),1)
        "Canny算子边缘检测"
        imgCanny = cv2.Canny(imgBlur,60,60)

        #检测形状，定出四个点
        "形状检测"
        locate , location = ShapeDetection(imgCanny,img)
        if(locate != [])&(location != []):
            #透视转换_坐标
            "设置变化图像大小"
            width,height = 520,520
            "将四个角点X的坐标转换"
            x = np.zeros(4)
            x[0] = locate[(0,0)]*locate[(0,0)] + locate[(0,1)]*locate[(0,1)]
            x[1] = locate[(1,0)]*locate[(1,0)] + locate[(1,1)]*locate[(1,1)]
            x[2] = locate[(2,0)]*locate[(2,0)] + locate[(2,1)]*locate[(2,1)]
            x[3] = locate[(3,0)]*locate[(3,0)] + locate[(3,1)]*locate[(3,1)]
            "创建数组，储存point点"
            point = np.zeros((4, 2))
            "找出X的最大值，赋值给point[2]"
            max_index, max_number = max(enumerate(x), key=operator.itemgetter(1))
            point[2] = location[max_index]
            "将X最大值变为0"
            location[max_index] = 0
            x[max_index] = 0
            max_index, max_number = max(enumerate(x), key=operator.itemgetter(1))
            if location[(max_index,0)] > 200:
                point[1] = location[max_index]
                location[max_index] = 0
                x[max_index] = 0
                max_index, max_number = max(enumerate(x), key=operator.itemgetter(1))
                point[3] = location[max_index]
                location[max_index] = 0
                x[max_index] = 0
                max_index, max_number = max(enumerate(x), key=operator.itemgetter(1))
                point[0] = location[max_index]
                location[max_index] = 0
                x[max_index] = 0
            else:
                point[3] = location[max_index]
                location[max_index] = 0
                x[max_index] = 0
                max_index, max_number = max(enumerate(x), key=operator.itemgetter(1))
                point[1] = location[max_index]
                location[max_index] = 0
                x[max_index] = 0
                max_index, max_number = max(enumerate(x), key=operator.itemgetter(1))
                point[0] = location[max_index]
                location[max_index] = 0
                x[max_index] = 0
            # print("point = ", point)
            # print("x = ",x)
            # print("location",location)

            #截取图像并画圆
            "进行透视操作"
            pts1 = np.float32(point)
            pts2 = np.float32([[0,0],[width,0],[width,height],[0,height]])
            matrix_K = cv2.getPerspectiveTransform(pts1,pts2)
            img_K = cv2.warpPerspective(img,matrix_K,(width,height))
            # cv2.imshow("img_k",img_K)
            img_K_copy = img_K.copy()
            "画圆"
            treasure_0 , flag =  drawcircles(img_K,img_K_copy)
            if flag != 999 :
                treasure = treasure_0
                imge = img_K
                # cv2.imwrite("../save/imge.png",imge)
                #对宝藏图进行二次处理
                "将宝藏按照人为顺序重新排列"
                treasure = ReTreasure(treasure,color)

                # Location = GetLocation(100,100)
                # print(Location)
                "为宝藏图添加起点和终点"
                if color == red :
                    treasure = np.insert(treasure,0,[380,20],axis=0)
                    treasure = np.append(treasure,[[20,380]],axis=0)
                else :
                    treasure = np.insert(treasure, 0, [ 20,  380],axis=0)
                    treasure = np.append(treasure, [[380,20]],axis=0)
                return treasure
                Treasures_Check.append(treasure)
                if(len(Treasures_Check) == 10):
                    for Treasure_Check in Treasures_Check:
                        if (Treasure_Check.all() == treasure.all()):
                            return treasure
                            Treasures_Check = []
                        else:
                            Treasures_Check.pop()
        cv2.waitKey(50)
def check(a):
    global b
    try:
        b = Point_Init(a)
        return 1
    except ValueError:
        return 0
    except IndexError:
        return 0
a = (Find_Treasure(0))
b = []
print(a)
while (check(a)==0):
    1
print(b)
