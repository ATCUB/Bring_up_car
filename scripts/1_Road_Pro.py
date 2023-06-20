# encoding: utf-8
import cv2
import numpy as np
import rospy
def detect_intersection_type():
    "获取照片"
    cap = cv2.VideoCapture(0)  # 0表示默认相机设备
    if not cap.isOpened():
        print("Failed to open camera")
        exit()
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        exit()
    while(True):
        # 读取图像并转换为灰度图
        imag = cap.read()[1]
        imag_shape = imag.shape
        image = imag[int(imag_shape[0]/4.0):imag_shape[0],0:imag_shape[1]]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 使用阈值化处理，使黑色循迹线变成白色，背景变成黑色
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # 使用膨胀操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.erode(thresh, kernel, iterations=1)
        dilated_1 = cv2.erode(dilated, kernel, iterations=1)
        dilated = cv2.erode(dilated_1, kernel, iterations=1)
        dilated = cv2.dilate(dilated, kernel, iterations=1)
        dilated = cv2.dilate(dilated, kernel, iterations=1)
        dilated = cv2.dilate(dilated, kernel, iterations=1)
        dilated = cv2.dilate(dilated, kernel, iterations=1)
        cv2.imshow("dilated",dilated)
        # 使用霍夫变换查找直线
        lines = cv2.HoughLinesP(dilated, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=100)
        H1 = 0.0
        H2 = 0.0
        W1 = 0.0
        W2 = 0.0
        H_Num_1 = 0.0
        H_Num_2 = 0.0
        W_Num_1 = 0.0
        W_Num_2 = 0.0
        # 获取图片大小
        P_shape = dilated.shape
        P_H = P_shape[0]
        P_W = P_shape[1]
        print(P_shape)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if abs(y2-y1) > 0.7*P_H:
                    H1 = H1 + abs(y2 - y1)
                    H_Num_1 = H_Num_1 + 1
                if ((abs(y2-y1) > 0.2*P_H) and (abs(y2-y1) < 0.7*P_H)):
                    H2 = H2 + abs(y2 - y1)
                    H_Num_2 = H_Num_2 + 1
                if abs(x2 - x1) > 0.7*P_W:
                    W1 = W1 + (x2 - x1)
                    W_Num_1 = W_Num_1 + 1
                if ((abs(x2 - x1) > 0.2*P_W)and(abs(x2 - x1) < 0.7*P_W)):
                    W2 = W2 + (x2 - x1) - (P_W/2.0 - x1)
                    W_Num_2 = W_Num_2 + 1
                cv2.imshow("image", image)
            cv2.waitKey(10)

            if H_Num_1 > H_Num_2 / 2.0:
                H_Num = H_Num_1
                H = H1
            else:
                H_Num = H_Num_2
                H = H2
            if W_Num_1 > W_Num_2 / 2.0:
                W_Num = W_Num_1
                W = W1
            else:
                W_Num = W_Num_2
                W = W2
            if ((H_Num!=0)and(W_Num!=0)):
                H = H / H_Num
                W = W / W_Num
                print("H = ", H)
                print("W = ", W)


                if(abs(H) >= 0.9*P_H):
                    if (abs(W) >= 0.9 * P_W):
                        print("这是十字路口")
                    elif((abs(W) >= 0.2*P_W)):
                        print("这是|-路口")
                    else:
                        print("这是-|路口")
                elif(abs(H) < 0.9*P_H):
                    if(abs(W) >= 0.9*P_W):
                        print("这是T型路口")
                    elif(W >= 0.2*P_W):
                        print("这是L右型路口")
                    elif (W <= 0.0):
                        print("这是L左型路口")



#image_path = "../photo/test20.png"
detect_intersection_type()
