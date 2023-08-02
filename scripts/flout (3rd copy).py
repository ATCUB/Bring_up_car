#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os
import threading
import math
import rospkg
import rospy
import serial
import time
import tf
import cv2 as cv
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from math import copysign, sqrt, pow
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Point
from dynamic_reconfigure.server import Server
from nav_msgs.msg import Odometry
# from dynamic_reconfigure.client import Client
from yahboomcar_bringup.cfg import FollowPIDnewConfig
import matplotlib.pyplot as plt
from typing import Any
from Tracks import GetNextDirctions
import struct
from std_msgs.msg import String
from pictureprocessing import Find_Treasure
from Tracks import FindTracks
treasure = Find_Treasure(0,"/home/jetson/pathshow_ws/src/pathshow/scripts/test14.png")
FindTracks(treasure)
dir_num = 1
direction2 = GetNextDirctions(dir_num)
#direction2 = [1,1,2,2,1,2,1,2,0,2,0,1,1,2]
# print("Start Direction is %d", direction2)
RAD2DEG = 180 / math.pi

# 运动组，例如0.2, -0.1, 0.2, 0.2, 180，表示以0.2的速度行走 -0.1m, 以0.2的速度行走 0.2m, 最后旋转180，顺序执行
# 数组末尾为旋转角度，如果为0表示不旋转，数组数量大于3,i且为奇数
Move_lastcorner = [0.2, 0.1, 180]

def timer_callback(event):
    rospy.loginfo('Timer triggered')

class MoveRobot:
    def __init__(self):
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))

        self.position = Point()

        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()

        self.x_start = self.position.x
        self.y_start = self.position.y

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber('/vel_raw', Twist, self.vel_callback)
        self.current_pos = 0
        self.previous_pos = 0
        self.measure_pos = 0
        self.measure_pos_end = 0
        self.measure_pos_last = 0
        self.move_cmd = Twist()
        self.first_call = 1
        self.first_call_move = 1
        self.start_move = True
        self.target_pos = 0
        self.real_vel = 0
        self.real_vel_last = 0
        self.delta_time = 0.1
        self.time_start_move = 0
        self.time_end_move = 0
        self.time_start_mesu = 0
        self.time_end_mesu = 0
        self.triger_last = 0
        self.tolerance = 0.01
        self.direction = 1
        self.speed = 0
        self.PID = [3.0, 0, 0.3]
        self.error = [0.0, 0.0, 0.0]

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)

    def vel_callback(self, msg):
        if not isinstance(msg, Twist):return
        self.real_vel_last =  self.real_vel
        self.real_vel = msg.linear.x

    def move(self, vel, req_distance):
        move_cmd = Twist()
        if self.first_call_move:
            self.position = self.get_position()
            self.x_start = self.position.x
            self.y_start = self.position.y  
            self.first_call_move = 0
            # print("First call")
            return False
        if self.start_move:
            # Get the current position from the tf transform between the odom and base frames
            self.position = self.get_position()
            # Compute the Euclidean distance from the target point
            distance = sqrt(pow((self.position.x - self.x_start), 2) +
                            pow((self.position.y - self.y_start), 2))

            # How close are we?
            req_distance = req_distance * -1
            if req_distance > 0:
                error = distance - req_distance
            elif req_distance < 0:
                error = (distance + req_distance) * -1
            else:
                return True
            
            self.error.append(error)
            if len(self.error) > 3:
                self.error.pop(0)

            if abs(self.error[-1]) > (abs(req_distance) + 0.05):
                self.position = self.get_position()
                self.x_start = self.position.x
                self.y_start = self.position.y
                self.cmd_vel.publish(Twist())  
                self.first_call_move = 1 
                print("error is increasing!!!")
                return True
            
            # Are we close enough?
            if not self.start_move or abs(self.error[-1]) < self.tolerance:
                self.position = self.get_position()
                self.x_start = self.position.x
                self.y_start = self.position.y
                self.cmd_vel.publish(Twist()) 
                self.speed = 0
                self.first_call_move = 1     
                # print("last distance: ", distance)
                return True         
            else:
                # If not, move in the appropriate direction
                if self.direction:
                    self.speed += self.PID[0] * (self.error[-1] - self.error[-2]) + self.PID[1] * self.error[-1] + self.PID[2] * (self.error[-1] - 2.0 * self.error[-2] + self.error[-3])
                    if self.speed > abs(vel):
                        self.speed = abs(vel)
                    elif self.speed < -1 * abs(vel):
                        self.speed = -1* abs(vel)
                    move_cmd.linear.x = self.speed
                else:
                    move_cmd.linear.y = copysign(vel, -1 * error)
                # print("distance: ", distance)
                # print("error: ", error)
                # print("speed: ", self.speed)
                self.cmd_vel.publish(move_cmd)
                return False
            # print ("time: {},distance: {},test_distance: {},position: {}".format(
            #     (start - end), distance, self.test_distance, self.position))
 
        
    def dis_measure(self, triger):
        if triger:
            self.triger_last = 1
            print("Measure is activated! ")
            if self.first_call:
                print("First call! ")
                self.first_call = 0
                self.time_start_mesu = rospy.get_time()
                self.time_end_mesu = self.time_start_mesu
                self.measure_pos_end = 0
                return 0
            self.time_end_mesu = self.time_start_mesu
            self.time_start_mesu = rospy.get_time()
            delta_time = self.time_start_mesu - self.time_end_mesu
            self.measure_pos_last = self.measure_pos
            self.measure_pos = self.measure_pos_last + (self.real_vel* 1.0 + self.real_vel_last* 0.0)  * delta_time
            return self.measure_pos
        else:
            if self.triger_last == 1:
                self.first_call = 1
                self.triger_last = 0
                self.time_end_mesu = self.time_start_mesu
                self.time_start_mesu = rospy.get_time()
                delta_time = self.time_start_mesu - self.time_end_mesu
                self.measure_pos = self.measure_pos_last + (self.real_vel* 1.0 + self.real_vel_last* 0.0)  * delta_time
                self.measure_pos_end = self.measure_pos
                self.measure_pos_last = 0
                self.measure_pos = 0
                print("Last clean! ")
                return self.measure_pos_end
            else:
                self.measure_pos_last = 0
                self.measure_pos = 0
                return None

class RotateRobot:
    def __init__(self):
        global direction2
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
                        #print(self.turn_count) 
                    else:
                        pass  
                    #print("Turn start yaw: ", self.real_yaw_start)
                    #print("Turn end yaw: ", self.real_yaw_end)
                    #print("Turn angle: ",  self.rotate_angle)
                    if abs(self.rotate_angle) >20:
                        self.position +=0
                    global follow_line
                   # if direction[Rotate_robo.position]==0 :
                    #    print("follow_line.for0:",follow_line.for0)  
                if self.position == len(direction2):
                    rospy.set_param('enable', True)
                    while(rospy.get_param('enable')):
                        pass
                #goal = rospy.get_param('goal')
                #if goal == 0:
                #    Cancel_Motion = 1 
                #    ROS_Ctrl.Rotate_Motion = 1
                #else:
                #    pass
                #print("goal:",goal)
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
        #maximum of Rotate velotity 5.0rad/s
        max_vel = 5
        #Rotate request
        if Rotate_cmd:
            #0-360 to (-180)-0-(180)
            self.Rotate_Real_Angle = self.real_yaw_filtered - 180
            # if firsat call this function, caculate the target angle
            if self.first_call:
                self.Rotate_Target = Rotate_angle + self.Rotate_Real_Angle
                while  self.Rotate_Target >= 180:
                    self.Rotate_Target -= 360
                while  self.Rotate_Target <= -180:
                    self.Rotate_Target  += 360
                self.first_call = 0
                print("Target accquired with", self.Rotate_Target)
            # Add velocity limit to avoid out of control
            error = self.Rotate_Target - self.Rotate_Real_Angle
            if  error >= 180:
                error -= 360
            elif  error <= -180:
                error  += 360
            self.Rotate_Angular = (error) * self.RotatePID[0] + self.rotate_angular * self.RotatePID[2]
#print("Error of angle accquired with%d ", self.Rotate_Target - self.Rotate_Real_Angle)
            error = self.Rotate_Target - self.Rotate_Real_Angle
            if abs(self.Rotate_Target - self.Rotate_Real_Angle) < 3:
                rospy.loginfo("Finished Rotate %d", Rotate_angle)
                self.Rotate_twist.angular.z = 0
            	self.Rotate_twist.linear.x = 0
                self.cmd_vel_pub.publish( self.Rotate_twist)
                self.first_call = 1
                return True
            else:
                self.Rotate_twist.angular.z = -self.Rotate_Angular
                #print("Vel published with %d ", self.Rotate_twist.angular.z)
            	self.Rotate_twist.linear.x = 0
                self.cmd_vel_pub.publish( self.Rotate_twist)
                return False
        else:
            rospy.loginfo("Rotate cmd is False!!!")
            return True

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
        global follow
        self.Cancel_Motion = 0
        self.Direction_index = 4
        self.Rotate_Motion = 0    
        self.Move_Motion = 0
        self.Measure_Switch = 0
        self.Direction_index_Last = 4
        self.Direction = {'Forward', 'Backward', 'TurnLeft', 'TurnRight', 'Stop'}
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_Joy = rospy.Subscriber('joy', Joy, self.buttonCallback)
        global follow_line

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
	        # follow_line.postion=0
            rospy.loginfo("Now Motion is %d", self.Cancel_Motion)
        if joy_data.buttons[1] == 1: 
            # follow_line.Follow_Twist.angular.z =  -7.0
            # follow_line.Follow_Twist.linear.y = 0.3
            # follow_line.pub_vel.publish(follow_line.Follow_Twist)
            time.sleep(0.3)
            # follow_line.Move_Stop()
            self.Direction_index += 1
            if self.Direction_index > 4:
                self.Direction_index = 0
            rospy.loginfo("Now Direction is %d", self.Direction_index)
        if joy_data.buttons[4] == 1:
	        # follow_line.Move_Stop()
            self.Rotate_Motion = 1
            rospy.loginfo("Rotate_Motion cmd received")
        if joy_data.buttons[3] == 1:
            self.Move_Motion = 1
            rospy.loginfo("Move_Motion cmd received")
        if joy_data.buttons[6] == 1:
            self.Measure_Switch ^= 1
            rospy.loginfo("Measure cmd received, Switch is %d",self.Measure_Switch)

    def cancel(self):
        self.sub_Joy.unregister()
        self.pub_cmdVel.unregister()

class FollowLine:
    def __init__(self):
        global Rotate_robo
        global Finish_Rotate_flag
        rospy.init_node("FollowLine", anonymous=False)
        self.dyn_update = False
        self.Start_state = True
        self.Data_Of_OPENMV = 0
        self.FollowPID = (60, 0, 20)
        self.linear_x = 1.0
        self.linear_y = 0
        self.turn=0
        self.direction_flag=0
        self.count = 0  
        self.last_count = 0
        self.angular_z = 0
        self.postion = 0
        self.error_of_z = 0
        self.FPS_count = 0
        self.start = 0
        self.end = 0
        self.flip = True
        self.stop_flag = 0
        self.next_first = 0
        self.error_pub = rospy.Publisher("/error_vel",Int16,queue_size=10)#发布速度话题
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
        self.ser_OPMV = serial.Serial('/dev/myopenmv', 115200, timeout=1)

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
        #self.PID_init()
        return config
    def Openmv_Data_Transmit(self, dir):
        data = struct.pack('fffffffi',0.0, 0.0, 0.0,3.0, 0.0, 0.5,dir,60)
        self.ser_OPMV.write(data)  

    def Openmv_Data_Receive(self):
        self.Openmv_Data_Rx = self.ser_OPMV.readline()
        #print('Data from openmv is ',    self.Openmv_Data_Rx)
        #print(self.Openmv_Data_Rx)
        global dir_num
        global direction2
        if self.Openmv_Data_Rx:
            if self.FPS_count ==  0:
                self.start = time.time()
            self.Openmv_Data_Rx = self.Openmv_Data_Rx.decode("utf-8")
            # Stop signal 
            if self.Openmv_Data_Rx[0] == 's' and self.turn==0:
                self.stop_flag = 1
            else:
                self.stop_flag = 0
            if 0:
                print("Stop cmd received!!!")
            else:
                # split data from openmv
                Data_Rx = self.Openmv_Data_Rx.split(",")
                try:
                    self.linear_y = float(Data_Rx[0]) / 200.0
                    self.angular_z = float(Data_Rx[1]) / 100.0
                    #print("z:",self.angular_z)
                # data
                    self.for0 = float(Data_Rx[2])
                # max black blob width in 6 ROI
                    self.Width = int(Data_Rx[3])
                except ValueError:
                        return
                except IndexError:
                        return
                # position confirm
                global Rotate_robo
                if abs(self.for0)>400 and direction2[Rotate_robo.position]==0 :
                   Rotate_robo.position +=1
                # error between black line and midpoint 
                error = float(Data_Rx[3])
                self.error_pub.publish(error)
                # linear y , unit: m/s, scale is 200
                self.linear_y = float(Data_Rx[0]) / 100.0
                self.Follow_Twist.angular.z =  -self.angular_z
                # linear_y maximum
                up = 0.25
                # linear_y minimum
                down = 0.10
                if self.linear_y > up:
                   self.linear_y = up
                if self.linear_y < -up:
                   self.linear_y = -up
                if self.linear_y < down and self.linear_y>-down:  
                   self.linear_y =0
                    #self.angular_z
                # turn behaviour detection, if max black blob width > 150, and turn  == 0, can be considered that turn behaviour is happening
                #print("width:",self.Width)
                if  self.Width > 115 and self.turn == 0:                  #out
                    # linear_x when turning
                    if  self.direction_flag == 0:
                        self.Follow_Twist.linear.x = -0.1
                        self.linear_x = -0.1
                    # set self.turn
                    self.turn =1
                    # reset self.count
                    self.count = 0
                    # linear_x when turning
                    # angular_z when turning (-5.0, 5.0)
                    if direction2[self.postion] ==1 or direction2[self.postion] ==4 :
                        self.Follow_Twist.angular.z = 10.0                   
                        self.Follow_Twist.linear.y = -0.2
                    if direction2[self.postion] ==2 or direction2[self.postion]==5:
                        self.Follow_Twist.angular.z =  -10.0
                        self.Follow_Twist.linear.y = 0.2

		    self.pub_vel.publish(self.Follow_Twist)
                    #print("direction:",direction2[self.postion])
                    #print("postion:",[self.postion])
                    rospy.set_param('beep', True)
                    self.pub_Buzzer.publish(True)
                    ## delay to hold turning state
                    if direction2[self.postion]!=0:
                       time.sleep(0.2)
		       self.ser_OPMV.reset_input_buffer()
	               self.ser_OPMV.reset_output_buffer()
                       self.Follow_Twist.linear.x = 0.5                            
                       self.linear_x =0.5
                    self.pub_Buzzer.publish(False)
                    #print("在转弯")
		    self.direction_flag = 0                
                    self.Follow_Twist.linear.y = 0.0
                    return
                # straight behaviour detection, if max black blob width < 105, and turn  == 1, can be considered that straight behaviour is happening
                if self.Width<110 and self.Width>90 and self.turn==1:
                    self.count+=1
                    if self.count >= 3:
                       self.turn = 0
                       self.linear_x =1.5
                       #print("在直线")
                       if not len(direction2) ==self.postion and self.next_first ==0:
                          self.postion += 1
                       else:
                          1  
                    #if len(direction2)-1 ==self.postion and self.turn == 0:                      
		if 1:
                    if len(direction2) ==self.postion:
                        if self.stop_flag == 1:
                            self.pub_vel.publish(Twist())
                            time.sleep(0.1)
                            movement_execute(Move_lastcorner)
                            self.direction_flag = 1
                            self.postion = 0
                            dir_num +=1
                            direction2 = GetNextDirctions(dir_num)
		            self.ser_OPMV.reset_input_buffer()
		            self.ser_OPMV.reset_output_buffer()
		            self.next_first =0
                        else:
                            self.linear_x =0.5                   
                if self.direction_flag ==1:
		    self.linear_x =0.8
                global Rotate_robo    
                self.Follow_Twist.linear.x = self.linear_x    
                self.Follow_Twist.linear.y =  self.linear_y      
                self.pub_vel.publish(self.Follow_Twist)
                self.FPS_count += 1
                if self.FPS_count ==  -1:
                    self.end = time.time()
                    self.FPS_count = 0
                    #rospy.loginfo("FPS: %f .", 20 / (self.end - self.start))
                    #rospy.loginfo("Angular.Z: %f .", self.angular_z)


    def Move_Stop(self):
        self.Follow_Twist.linear.x = 0
        self.Follow_Twist.linear.y = 0
        self.Follow_Twist.linear.z = 0
        self.Follow_Twist.angular.x = 0
        self.Follow_Twist.angular.y = 0
        self.Follow_Twist.angular.z = 0
        self.pub_vel.publish(self.Follow_Twist)

# param[in]: velocity, distance, angle
# param[out]: True, finished movement group
# param[out]: False, finishing movement group
def movement_execute(mov_grop):
    mov_rate = rospy.Rate(10)
    rotate_rate = rospy.Rate(100)
    global follow_line
    if mov_grop != None:
        length_grop = len(mov_grop)
        if length_grop % 2 != 1:
            rospy.logwarn("mov_grop is not complete, or too many args!!!")
            return False
        else:
            for i in range((length_grop - 1)/ 2):
                while not Move_robo.move(mov_grop[i*2], mov_grop[i*2+1]) and not rospy.is_shutdown():
                    #print("Movement Group num %d is handled\r\n",i+1)
                    mov_rate.sleep()
            while not Rotate_robo.Robot_Rotate((mov_grop[length_grop - 1] != 0), mov_grop[length_grop - 1]) and not rospy.is_shutdown():
                rotate_rate.sleep()
            follow_line.ser_OPMV.reset_input_buffer()
            follow_line.ser_OPMV.reset_output_buffer()                
            rospy.loginfo("Movement executed successfully!")
            return True

if __name__ == '__main__':
    rospy.init_node("FollowLine", anonymous=False)
    ROS_Ctrl = ROSCtrl()         #init class ROSCtrl
    Rotate_robo = RotateRobot()
    follow_line = FollowLine()
    Move_robo = MoveRobot()
    #follow_line.Openmv_Data_Transmit(direction[Rotate_robo.position])
    Finish_Measure_flag = False
    rate = rospy.Rate(100)
    follow_line.Openmv_Data_Transmit(3)
    rospy.loginfo("Init successfully!")
    follow_line.ser_OPMV.reset_input_buffer()
    follow_line.ser_OPMV.reset_output_buffer() 
    while not rospy.is_shutdown():
        if not ROS_Ctrl.Cancel_Motion:
            follow_line.Openmv_Data_Receive()
            if ROS_Ctrl.Move_Motion:
                movement_execute(Move_shortline)
            if ROS_Ctrl.Measure_Switch:
                print("distance is ",Move_robo.dis_measure(1))
                Finish_Measure_flag = True
            else:
                if Finish_Measure_flag:
                    Finish_Measure_flag = False
                    Move_robo.dis_measure(0)
                else:
                    pass
            ROS_Ctrl.Move_Motion = 0
        rate.sleep()
    rospy.loginfo("Follow Line Node Exit!")


# if __name__ == '__main__':
#     rospy.init_node("FollowLine", anonymous=False)
#     ROS_Ctrl = ROSCtrl()         #init class ROSCtrl
#     Rotate_robo = RotateRobot()
#     Move_robo = MoveRobot()
#     #follow_line.Openmv_Data_Transmit(3)
#     Finish_Measure_flag = False
#     rate = rospy.Rate(100)
#     rospy.loginfo("Init successfully!")
#     while not rospy.is_shutdown():
#         if not ROS_Ctrl.Cancel_Motion:
#             if Rotate_robo.position_last != Rotate_robo.position:
#                 Rotate_robo.position_last = Rotate_robo.position
#             else:
#                 Rotate_robo.position_last = 0
#                 Rotate_robo.position = 0	        
#             if ROS_Ctrl.Measure_Switch:
#                 Finish_Measure_flag = True
#                 if abs(Move_robo.dis_measure(1) - 0.15) < 0.01:
#                     ROS_Ctrl.Measure_Switch = 0
#                     ROS_Ctrl.Move_Motion = 1
#                     continue
#             else:
#                 if Finish_Measure_flag:
#                     Finish_Measure_flag = False
#                     Move_robo.dis_measure(0)
#                 else:
#                     pass
#             if ROS_Ctrl.Move_Motion:
#                  if movement_execute(Move_shortline):
#                     ROS_Ctrl.Move_Motion =0
#         ROS_Ctrl.Move_Motion = 0
#         rate.sleep()
#     rospy.loginfo("Follow Line Node Exit!")
