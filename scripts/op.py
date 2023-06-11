#!/usr/bin/env python
# encoding: utf-8
import struct
import serial
import rospy
import numpy as np
import time
import gzip
import math
from std_msgs.msg import Int16
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from yahboomcar_bringup.cfg import  FollowPIDConfig# 导入自己的参数配置文件
from Tracks import GetNextDirctions
from pictureprocessing import Find_Treasure
from Tracks import FindTracks
position = 0


class RotateRobot:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('/imu/imu_data', Imu, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.origin_yaw = 0.0
        self.real_yaw = 0.0
        self.real_yaw_now = 0.0
        self.real_yaw_last = 0.0
        self.target_yaw = 90.0
        self.init_count = 0
        self.first_call = 1
        self.Detect_count = 0
        self.imu_direction = 0
        self.imu_direction_last = 0
        self.position=0

    def imu_callback(self, msg):
        if not isinstance(msg, Imu): return
        if(self.init_count <  5):
            self.init_count += 1
            self.origin_yaw = self.quaternion_to_yaw(msg.orientation)
        else:
            self.real_yaw = self.quaternion_to_yaw(msg.orientation) - self.origin_yaw
            if self.real_yaw > 360:
                self.real_yaw -= 360
            elif self.real_yaw < 0:
               self.real_yaw  += 360
            #first time call, init last_yaw
            if(self.first_call):
                self.real_yaw_now = self.real_yaw
                self.real_yaw_last = self.real_yaw
                self.first_call = 0
                return
            #straight line detect, if or not running on the straight line
            if(self.Detect_count < 30):
                self.real_yaw_last = self.real_yaw_now
                self.real_yaw_now = self.real_yaw
                self.Detect_count  += 1
                if abs(self.real_yaw_last - self.real_yaw_now) < 10:
                    self.imu_direction = 0
                else :
                    self.imu_direction = 1
                if self.imu_direction_last ==1 and self.imu_direction == 0:
                    self.position += 1
                self.imu_direction_last  = self.imu_direction
                print(self.position)
            else:
                self.Detect_count = 0

    def quaternion_to_yaw(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        yaw = math.degrees(yaw)
        #print(self.strange)
        if yaw < 0:
            yaw  += 360
        return yaw

if 1:
    RotateRobot()
    linear_x = 0.1
    p1 = 0
    p2 = 0
    i1 = 0
    i2 = 0
    d1 = 0
    d2 = 0
treasure = Find_Treasure(0,"/home/jetson/pathshow_ws/src/pathshow/scripts/test14.png")
FindTracks(treasure)
dir = GetNextDirctions(1)
dir[-2] = 4
print(dir)
flag = 0
#pid初始化
def callback(config, level):
    # 这里是当参数被修改后的回调函数，可以在这里进行一些操作
    if 1:#定义PID参数和线速度，flag暂时没用，打算用来控制小车停止/前进
        global p2
	global i2
        global d2
	global flag
        linear = config['linear']
        p1 = config['p1']
        i1 = config['i1']
        d1 = config['d1']
        p2 =  config["p2"]
        i2 =  config["i2"]
        d2 =  config["d2"]
        #flag = config["direction"]
        flag = dir[position]
        state_in = config["state"]
    data = struct.pack('fffffff', p1, i1, d1, p2, i2, d2,flag)
    ser.write(data)
    global linear_x#声明线速度为全局变量，以便后面能用
    linear_x = linear
    global state#
    state = state_in
    #以上全是发送过程
    #pid和flag参数赋予
    rospy.loginfo("New config: %s", str(config))
    return config

rospy.init_node('op',anonymous=True)
rate = rospy.Rate(100)
vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)#发布速度话题
error_pub = rospy.Publisher("/error_vel",Int16,queue_size=10)#发布速度话题
twist=Twist()
ser = serial.Serial(port="/dev/ttyUSB1",baudrate=115200,timeout=0.5)#打开串口
if ser.is_open :
    print("open")
else:
    print("error")
Server(FollowPIDConfig, callback)#开启参数服务器，每次在rqt中更改参数就会进行一次call
error_array = []
stay=0
last_len =50
ary = 0
num=0
flag1=0
rospy.set_param('enable', True)
stop = 0
turn = 0
uart_count = 0
#if __name__=="__main__":

while not rospy.is_shutdown() & ser.is_open :
    if 0:#忽视这里
        print("初始化发送PID")
        p1 = 0.075
        i1 = 0
        d1 = 0.001
        p2 =  1.79
        i2 =  0
        d2 =  0.015
        flag = 4
    	data = struct.pack('fffffff', 0, 0, 0, p2, i2, d2,flag)
    	ser.write(data)
	print("初始化PID完毕")
	init = 0
    else:#永远进这个分支
	#print(uart_count)
	#uart_count = uart_count + 1
        num+=1
        vel = ser.readline()#读取串口中一行信息
        if vel:#如果有信息传过来
            if flag1 == 0:
                start = time.time()
                flag1 = 1
           
            elif flag1 == 1:
                flag1= 0
                end = time.time()
                fps = 1 / ( end - start )
                #print("FPS:",fps)
            vel = vel.decode("utf-8")#对传过来的数据解码
            if vel[0] == "s" or state == 0:#如果以s开头，就是stop
                twist.angular.z = 0
                twist.linear.y = 0
                twist.linear.x = 0
                if state == 0:
                    position =0
		if stop == 0:
                    vel_pub.publish(twist)
		    stop =1
                else:
	            1
	    elif dir[position] == 0 and vel[0]=='v':
		position = position+1
		flag = dir[position]
		data = struct.pack('fffffff', p1, i1, d1, p2, i2, d2,flag)
    		ser.write(data)   
            else:#说明是PID的输出
                stop = 0
		print("now position:",position,"now direction:",flag)	
                a = vel.split(",")
                y = float(a[0])
                z = float(a[1])
                error  =  float(a[2])
                error_pub.publish(error)
                z = -z#改变极性使得方向正确
		#获得yz两个速度
                print("y=",y)
                if y > 5:#限幅
                    y = 5
		print("z=",z)
		if abs(z)>200 and abs(z)<500 and (dir[position]==1 or dir [position] ==2):
		    z = 1.5*z
		if abs(z)>800:
		    #z = 1.5*z		
		    turn = 1#在转弯
                    count = 0
		#if abs(z) <500 and abs(z) >200:
		  #  z = 1.5*z
                if abs(z)>350 and dir[position]==0:
		    turn = 1
	        if turn == 1 and abs(z)<50:
                    count = count+1
                    if count == 3:
                        turn = 0#结束转弯
                       # position = position+1
                        flag = dir[position]
		        data = struct.pack('fffffff', p1, i1, d1, p2, i2, d2,flag)
    		        ser.write(data)       
                twist.angular.z = z/100
                twist.linear.y = y/20
                twist.linear.x = linear_x
                vel_pub.publish(twist)#发布速度
                #下面都是一些调试的接口，包括误差存储和误差变化图，已经废弃         
                #last_len =len(error_array)
                # if num %10 == 0  and num!= 0:
                #     end = time.time()
                #     fps = 1 / ( end - start )
                #     print("FPS")
                #     print(fps)
                #     print("start" + str(start))
                #     print("end" + str(end))
                #if  len(error_array)-last_len >= 10:
                #print(len(error_array)) 
                    #plt.ion()
                    #last_len = len(error_array)
                    #ary = np.array(error_array)
                    #np.savetxt("error.txt",ary)  
    		    #ary = np.array(error_array)
                    #np.savetxt("error.txt",ary)
                    #plt.axis([0,len(error_array)+20,-300,300])
                    #plt.plot(range(len(error_array)),error_array)
                    #plt.show()
                    #plt.pause(0.05)

		    

    #stay = rospy.wait_for_message('/my_topic', Int16, timeout = None)

    rate.sleep()



        


