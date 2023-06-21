#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os
import threading
import math
import rospkg
import rospy
import serial
import time
import cv2 as cv
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
# from dynamic_reconfigure.client import Client
from yahboomcar_bringup.cfg import FollowPIDnewConfig
import matplotlib.pyplot as plt
from typing import Any
from Tracks import GetNextDirctions
import struct
from pictureprocessing import Find_Treasure
from Tracks import FindTracks

treasure = Find_Treasure(0,"/home/jetson/pathshow_ws/src/pathshow/scripts/test14.png")
FindTracks(treasure)
direction = GetNextDirctions(1)
direction = [0,4,0,3.5,0,3.5,4,3.5]
print("Start Direction is %d", direction)
RAD2DEG = 180 / math.pi
    

class RotateRobot:
    def __init__(self):
        global direction
        self.imu_sub = rospy.Subscriber('/imu/imu_data', Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Rotate_twist = Twist()
        self.origin_yaw = 0.0
        self.real_yaw = 0.0
        self.real_yaw_window = []
        self.window_size = 3
        self.real_yaw_last = 0
        self.real_yaw_filtered_last = 0
        self.real_yaw_start = None
        self.real_yaw_end = None
        self.real_yaw_filtered = 0
        self.real_yaw_delta = 0
        self.rotate_angle = 0
        self.rotate_angular = 0
        self.target_yaw = 90.0
        self.init_count = 0
        self.first_call = 1
        self.Detect_count = 10
        self.imu_direction = 0
        self.turn_flag=1
        self.turn_count = 0
        self.RotatePID = [0.027, 0, 0]
        self.Rotate_Target = 0
        self.Rotate_Angular = 0
        self.Rotate_Real_Angle = 0
        self.first_call = 1
        self.position = 0
        self.position_last = 0

    def vel_callback(self, msg):
        if not isinstance(msg, Twist): return
        self.rotate_angular = -msg.angular.z

    def imu_callback(self, msg):
        if not isinstance(msg, Imu): return
        #first 20 datas for stabilizing data, and check the origin data
        if(self.init_count <  20):
            self.init_count += 1
            self.origin_yaw = self.quaternion_to_yaw(msg.orientation)
        else:
            #real yaw = yaw - origin yaw
            self.real_yaw = self.quaternion_to_yaw(msg.orientation) - self.origin_yaw
            # make the real yaw in 0 to 360
            if self.real_yaw > 360:
                self.real_yaw -= 360
            elif self.real_yaw < 0:
               self.real_yaw  += 360

            # Turning detect, slide window filter
            if abs(self.real_yaw_last - self.real_yaw) > 180:    # Here is if data change in 0 to 360, window be cleared for newest yaw data
                self.real_yaw_window = []
            self.real_yaw_last = self.real_yaw_last - self.real_yaw
            self.real_yaw_window.append(self.real_yaw)
            if len(self.real_yaw_window) > self.window_size:
                self.real_yaw_window.pop(0)
            self.real_yaw_filtered = sum(self.real_yaw_window) / len(self.real_yaw_window)
            
            # print("self.real_yaw_last: ", self.real_yaw_last)
            self.real_yaw_delta = self.real_yaw_filtered - self.real_yaw_filtered_last

            # make sure the delta is right in 0 to 360
            if self.real_yaw_delta > 180:
                self.real_yaw_delta -= 360
            elif self.real_yaw_delta < -180:
                self.real_yaw_delta +=360
            # print("self.real_yaw_filtered: ", self.real_yaw_filtered)
            # print("self.real_yaw_delta: ", self.real_yaw_delta)
            # if delta > 5, start recording the angle of start and end
            if abs(self.real_yaw_delta) > 5:
                if self.real_yaw_start is None:
                    self.real_yaw_start = self.real_yaw_filtered_last
                self.real_yaw_end = self.real_yaw_filtered
            else:
                if self.real_yaw_start is not None and self.real_yaw_end is not None:
                    self.rotate_angle = self.real_yaw_end - self.real_yaw_start
                    # # make sure the rotate_angle is right in 0 to 360
                    if self.rotate_angle > 180:
                        self.rotate_angle -= 360
                    elif self.rotate_angle < -180:
                        self.rotate_angle +=360
                    #record the angle of car rotating
                    if abs(self.rotate_angle) > 60.0 :
                        self.turn_count +=1
                        print(self.turn_count) 
                    else:
                        pass  
                    print("Turn start yaw: ", self.real_yaw_start)
                    print("Turn end yaw: ", self.real_yaw_end)
                    print("Turn angle: ",  self.rotate_angle)
                    if abs(self.rotate_angle) >20:
                        self.position +=0
                    global follow_line
                   # if direction[Rotate_robo.position]==0 :
                    #    print("follow_line.for0:",follow_line.for0)  
                if self.position == len(direction):
                    rospy.set_param('enable', True)
                    while(rospy.get_param('enable')):
                        pass
                goal = rospy.get_param('goal')
                if goal == 0:
                    Cancel_Motion = 1 
                    ROS_Ctrl.Rotate_Motion = 1
                else:
                    pass
                print("goal:",goal)
                self.real_yaw_start = None
                self.real_yaw_end = None
            self.real_yaw_filtered_last = self.real_yaw_filtered

    def quaternion_to_yaw(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        yaw = math.degrees(yaw)
        if yaw < 0:
            yaw  += 360
        return yaw

    def Robot_Rotate(self, Rotate_cmd, Rotate_angle):
        max_vel = 5
        if Rotate_cmd:
            self.Rotate_Real_Angle = self.real_yaw_filtered - 180
            if self.first_call:
                self.Rotate_Target = Rotate_angle + self.Rotate_Real_Angle
                if  self.Rotate_Target > 180:
                    self.Rotate_Target -= 360
                elif  self.Rotate_Target < -180:
                    self.Rotate_Target  += 360
                self.first_call = 0
                print("Target accquired with%d ", self.Rotate_Target)
            # Add velocity limit to avoid out of control
            self.Rotate_Angular = (self.Rotate_Target - self.Rotate_Real_Angle) * self.RotatePID[0] + self.rotate_angular * self.RotatePID[2]
            #print("Error of angle accquired with%d ", self.Rotate_Target - self.Rotate_Real_Angle)
            if abs(self.Rotate_Angular) > max_vel:
                self.Rotate_Angular = max_vel if self.Rotate_Angular > 0 else -max_vel
            self.Rotate_twist.angular.z = -self.Rotate_Angular
            #print("Vel published with %d ", self.Rotate_twist.angular.z)
            self.cmd_vel_pub.publish( self.Rotate_twist)
            if abs(self.Rotate_Target - self.Rotate_Real_Angle) < 0.9:
                rospy.loginfo("Finished Rotate %d", Rotate_angle)
                self.first_call = 1
                return True
            else:
                return False

    # def run(self):
    #     while not rospy.is_shutdown():

    #         else:
    #             self.twist.angular.z = 0.0
    #             rospy.set_param('enable',0)
    #         self.cmd_vel_pub.publish(self.twist)
    #         self.rate.sleep()


#class controller include of sub_joy and  pub_cmdVel
class ROSCtrl:
    def __init__(self):
        self.Cancel_Motion = 0
        self.Direction_index = 4
        self.Rotate_Motion = 0
        self.Direction_index_Last = 4
        self.Direction = {'Forward', 'Backward', 'TurnLeft', 'TurnRight', 'Stop'}
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_Joy = rospy.Subscriber('joy', Joy, self.buttonCallback)

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        self.user_jetson(joy_data)

    def user_jetson(self, joy_data):
        '''
        :jetson joy_data:
            axes 8: [0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
            R2: axes[4] cancel_nav
            L2: axes[5]
            buttons 15:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            A: buttons[0]
            B: buttons[1]
            X: buttons[3]
            Y: buttons[4]
            L1: buttons[6]
            R1: buttons[7] RGBLight
            SELECT: buttons[10]
            START: buttons[11] Buzze
        '''
        if joy_data.buttons[0] == 1:
            self.Cancel_Motion ^= 1
            rospy.loginfo("Now Motion is %d", self.Cancel_Motion)
        if joy_data.buttons[1] == 1:  
            self.Direction_index += 1
            if self.Direction_index > 4:
                self.Direction_index = 0
            rospy.loginfo("Now Direction is %d", self.Direction_index)
        if joy_data.buttons[4] == 1:
            self.Rotate_Motion = 1
            rospy.loginfo("Rotate_Motion cmd received")

    def cancel(self):
        self.sub_Joy.unregister()
        self.pub_cmdVel.unregister()

class FollowLine:
    def __init__(self):
        global Rotate_robo
        rospy.init_node("FollowLine", anonymous=False)
        self.dyn_update = False
        self.Start_state = True
        self.Data_Of_OPENMV = 0
        self.FollowPID = (60, 0, 20)
        self.linear_x = 0.3
        self.linear_y = 0
        self.turn=0
        self.count = 0  
        self.angular_z = 0
        self.error_of_z = 0
        self.FPS_count = 0
        self.start = 0
        self.end = 0
        self.flip = True
        self.Track_state = 'waiting'
        self.Buzzer_state = False
        self.Send_Count = 0
        Server(FollowPIDnewConfig, self.dynamic_reconfigure_callback)
        # self.dyn_client = Client("FollowLine", timeout=60)
        self.ros_ctrl = ROSCtrl()
        self.Follow_Twist = Twist()
        self.pub_Buzzer = rospy.Publisher("/Buzzer", Bool, queue_size=1)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_IR = rospy.Subscriber("/serial_IRdata_msg", Int16, queue_size=1)
        self.sub_vel = rospy.Subscriber("/pub_vel", Twist, queue_size=1)
        self.ser_OPMV = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        self.sub_IR.unregister()
        self.sub_vel.unregister()
        self.pub_Buzzer.unregister()
        print("Shutting down this node.\r\n")

    def Reset(self):
        self.Track_state = 'waiting'
        rospy.loginfo("Reset Successfully! Robot is waiting.")

        
    def dynamic_reconfigure_callback(self, config, level):
       # self.linear_x = config['linear_x']
        self.FollowPID = (config['Kp'], config['Ki'], config['Kd'])
        self.PID_init()
        return config

    def Openmv_Data_Receive(self):

        self.Openmv_Data_Rx = self.ser_OPMV.readline()

        #print(self.Openmv_Data_Rx)
        if self.Openmv_Data_Rx:
            if self.FPS_count ==  0:
                self.start = time.time()
            self.Openmv_Data_Rx = self.Openmv_Data_Rx.decode("utf-8")
            # Stop signal 
            if self.Openmv_Data_Rx[0] == 's' :
                self.Move_Stop()
           # elif self.Openmv_Data_Rx[0] == 'L':
             #   if self.turn==0:
                       # self.Follow_Twist.linear.x = 0.0
                       # self.Follow_Twist.angular.z =  10.0
                       # self.pub_vel.publish(self.Follow_Twist)
                       # time.sleep(0.2)
                       # self.turn = 1
                       # print("左转")
            #elif self.Openmv_Data_Rx[0] == 'R':
               # if self.turn==0:
                      #  self.Follow_Twist.linear.x = 0.0
                     #   self.Follow_Twist.angular.z =  -10.0
                     #   self.pub_vel.publish(self.Follow_Twist)
                     #   time.sleep(0.1)
                      #  self.turn = 1
                       # print("右转")
         #   elif self.Openmv_Data_Rx[0] == 'T':
               # if self.turn==0:
                     #   self.Follow_Twist.linear.x = 0.0
                     #   self.Follow_Twist.angular.z =  -10.0
                      #  self.pub_vel.publish(self.Follow_Twist)
                       # time.sleep(0.1)
                     #   self.turn = 1
                      #  print("T右转")
           # elif self.Openmv_Data_Rx[0] == "D":
                #pass
            # Normal running
            else:
                # split data from openmv
                Data_Rx = self.Openmv_Data_Rx.split(",")
                # angular z , unit: rad/s, scale is 100
                self.angular_z = float(Data_Rx[1]) / 100.0
                # data
                self.for0 = float(Data_Rx[2])
                # max black blob width in 6 ROI
                self.Width = float(Data_Rx[3])
                # position confirm
                global Rotate_robo
                if abs(self.for0)>400 and direction[Rotate_robo.position]==0 :
                   Rotate_robo.position +=1
                   print("fuck")    
                # error between black line and midpoint 
                self.error_of_z = float(Data_Rx[2])
                # linear y , unit: m/s, scale is 200
                self.linear_y = float(Data_Rx[0]) / -200.0
                # linear_y maximum
                up = 0.25
                # linear_y minimum
                down = 0.05
              #  if self.angular_z>0:
                 #   self.angular_z = self.angular_z*0.5
                    #self.linear_y = self.linear_y*4.0
                # limit linear y (0.05 to 0.25 m/s)
                if self.linear_y > up:
                   self.linear_y = up
                if self.linear_y < -up:
                   self.linear_y = -up
                if self.linear_y < down and self.linear_y>-down:  
                   self.linear_y =0
                    #self.angular_z
                # turn behaviour detection, if max black blob width > 150, and turn  == 0, can be considered that turn behaviour is happening
                if  self.Width > 150 and self.turn == 0:
                    # linear_x when turning
                    self.linear_x = 0
                    # set self.turn
                    self.turn =1
                    # reset self.count
                    self.count = 0
                    # linear_x when turning
                    self.Follow_Twist.linear.x = 0.0
                    # angular_z when turning (-5.0, 5.0)
                    self.Follow_Twist.angular.z =  10.0
                    self.pub_vel.publish(self.Follow_Twist)
                    # delay to hold turning state
                    time.sleep(0.1)
                    print("在转弯") 
                # straight behaviour detection, if max black blob width < 105, and turn  == 1, can be considered that straight behaviour is happening
                if self.Width<105 and self.turn==1:
                    self.linear_x = 0.3
                    print("count:",self.count)
                    if self.count > 5:
                       self.turn = 0
                    print("在直线")
               # print("linear_z",self.linear_y)
		    #print("linear_z",self.angular_z)
                global Rotate_robo
                if direction[Rotate_robo.position] == 3 or direction[Rotate_robo.position] ==0 or self.turn==1:
                   self.Follow_Twist.linear.y = self.linear_y
                else:
                   self.Follow_Twist.linear.y = 0
                self.Follow_Twist.linear.x = self.linear_x
                self.Follow_Twist.angular.z =  -self.angular_z
                self.pub_vel.publish(self.Follow_Twist)
                self.FPS_count += 1
                if self.FPS_count ==  -1:
                    self.end = time.time()
                    self.FPS_count = 0
                    rospy.loginfo("FPS: %f .", 20 / (self.end - self.start))
                    rospy.loginfo("Angular.Z: %f .", self.angular_z)


    def Move_Stop(self):
        self.Follow_Twist.linear.x = 0
        self.Follow_Twist.linear.y = 0
        self.Follow_Twist.linear.z = 0
        self.Follow_Twist.angular.x = 0
        self.Follow_Twist.angular.y = 0
        self.Follow_Twist.angular.z = 0
        self.pub_vel.publish(self.Follow_Twist)
    
    def Openmv_Data_Transmit(self, dir):
        data = struct.pack('fffffff',0.0, 0.0, 0.00,2.4, 0.0, 0.35,dir)
        self.ser_OPMV.write(data)          	


if __name__ == '__main__':
    rospy.init_node("FollowLine", anonymous=False)
    follow_line = FollowLine()
    ROS_Ctrl = ROSCtrl()         #init class ROSCtrl
    Rotate_robo = RotateRobot()
    if follow_line.ser_OPMV.is_open:
        rospy.loginfo("OPmv module serial port --/dev/ttyUSB1 open successfully!")
    rospy.loginfo("Init successfully!")
    #follow_line.Openmv_Data_Transmit(direction[Rotate_robo.position])
    Finish_Rotate_flag = False
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        if not ROS_Ctrl.Cancel_Motion:
            follow_line.Openmv_Data_Receive()
            #rospy.loginfo("Direction[%d] is %d", Rotate_robo.position,direction[Rotate_robo.position])
            if Rotate_robo.position_last != Rotate_robo.position:
                Rotate_robo.position_last = Rotate_robo.position
                #follow_line.Openmv_Data_Transmit(direction[Rotate_robo.position])
                follow_line.Openmv_Data_Transmit(3)
               # rospy.loginfo("Direction[%d] is %d", Rotate_robo.position,direction[Rotate_robo.position])
            else:
                Rotate_robo.position_last = 0
                Rotate_robo.position = 0	        
            if ROS_Ctrl.Rotate_Motion:
                    if not Finish_Rotate_flag:
                        Finish_Rotate_flag = Rotate_robo.Robot_Rotate(1, 30)
                    else:
                        ROS_Ctrl.Rotate_Motion = 0
                        Finish_Rotate_flag = False
        rate.sleep()
    rospy.loginfo("Follow Line Node Exit!")
