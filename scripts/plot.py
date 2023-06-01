#!/usr/bin/env python
# encoding: utf-8
#import rospy
#from nav_msgs.msg import Odometry

#rospy.init_node("odom_listener")

#def  odom_callback(msg):
#    postion = msg.pose.pose.position
#    print("Current position: x={}, y={}, z={}".format(postion.x,postion.y,postion.z))
#odom_sub = rospy.Subscriber("/odom",Odometry,odom_callback)
#odom_pub = rospy.Publisher("/odom",Odometry,queue_size=10)
#new_odom_msg =Odometry()
#new_odom_msg.pose.pose.position.x = 1.0
#new_odom_msg.pose.pose.position.y = 2.0
#new_odom_msg.pose.pose.position.z = 3.0
#a = 0
#rate = rospy.Rate(100)
#while 1:
#	a = a+1
#	if a%100 == 0:
#		odom_pub.publish(new_odom_msg)
#       rate.sleep()
import cv2

# 读取图像
img = cv2.imread('image.jpg')

# 将图像转换为灰度图像
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 使用二值化技术将图像转换为黑白图像
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# 使用形态学操作来消除噪声和填充空洞
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

# 使用轮廓检测技术来检测十字，L子，T字的轮廓
contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 遍历轮廓并根据其形状和位置进行分类
for cnt in contours:
    # 计算轮廓的周长和面积
    perimeter = cv2.arcLength(cnt, True)
    area = cv2.contourArea(cnt)
    
    # 计算轮廓的近似多边形
    approx = cv2.approxPolyDP(cnt, 0.01 * perimeter, True)
    
    # 根据轮廓的形状和位置进行分类
    if len(approx) == 12:
        print('十字')
    elif len(approx) == 8:
        if area > 100:
            print('L子')
    elif len(approx) == 7:
        if area > 100:
            print('T字')




