#!/usr/bin/env python3
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

RAD2DEG = 180 / math.pi

#limit excute num
class LimitedRun(object):
    run_dict = {}

    def __init__(self, tag: Any = 'default', limit: int = 1):
        self.tag = tag
        self.limit = limit

    def __enter__(self):
        if self.tag in LimitedRun.run_dict.keys():
            LimitedRun.run_dict[self.tag] += 1
        else:
            LimitedRun.run_dict[self.tag] = 1
        return LimitedRun.run_dict[self.tag] <= self.limit

    def __exit__(self, exc_type, exc_value, traceback):
        return
    

class RotateRobot:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('/imu/imu_data', Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.Rotate_twist = Twist()
        self.origin_yaw = 0.0
        self.real_yaw = 0.0
        self.real_yaw_window = []
        self.window_size = 15
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
        rospy.init_node("FollowLine", anonymous=False)
        self.dyn_update = False
        self.Start_state = True
        self.Data_Of_OPENMV = 0
        self.FollowPID = (60, 0, 20)
        self.linear_x = 0.4
        self.linear_y = 0
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
        self.PID_init()
        self.ros_ctrl = ROSCtrl()
        self.Follow_Twist = Twist()
        self.pub_Buzzer = rospy.Publisher("/Buzzer", Bool, queue_size=1)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_IR = rospy.Subscriber("/serial_IRdata_msg", Int16, queue_size=1)
        self.sub_vel = rospy.Subscriber("/pub_vel", Twist, queue_size=1)
        self.ser_OPMV = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        self.sub_IR.unregister()
        self.sub_vel.unregister()
        self.pub_Buzzer.unregister()
        print("Shutting down this node.\r\n")

    def Reset(self):
        self.PID_init()
        self.Track_state = 'waiting'
        rospy.loginfo("Reset Successfully! Robot is waiting.")

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.FollowPID[0] / 1.0 , 0],
            [self.FollowPID[1] / 1.0 , 0],
            [self.FollowPID[2] / 1.0 , 0])
        
    def dynamic_reconfigure_callback(self, config, level):
        self.linear_x = config['linear_x']
        self.FollowPID = (config['Kp'], config['Ki'], config['Kd'])
        self.PID_init()
        return config

    def Openmv_Data_Receive(self):
        try:
            self.Openmv_Data_Rx = self.ser_OPMV.readline()
        except:
            return
        #print(self.Openmv_Data_Rx)
        if self.Openmv_Data_Rx:
            if self.FPS_count ==  0:
                self.start = time.time()
            self.Openmv_Data_Rx = self.Openmv_Data_Rx.decode("utf-8")
            if self.Openmv_Data_Rx[0] == 's' :
                self.Move_Stop()
            else:
                Data_Rx = self.Openmv_Data_Rx.split(",")
                self.linear_y = float(Data_Rx[0])
                self.angular_z = float(Data_Rx[1]) / 100
                self.error_of_z = float(Data_Rx[2])

                self.Follow_Twist.angular.x = self.linear_x
                self.Follow_Twist.angular.z =  self.angular_z
                self.pub_vel.publish(self.Follow_Twist)
                self.FPS_count += 1
                if self.FPS_count == 100:
                    self.end = time.time()
                    self.FPS_count = 0
                    rospy.loginfo("FPS: %f .", 100 / (self.end - self.start))
                    rospy.loginfo("Angular.Z: %f .", self.angular_z)


    def Move_Stop(self):
        self.Follow_Twist.angular.x = 0
        self.Follow_Twist.angular.y = 0
        self.Follow_Twist.angular.z = 0
        self.pub_vel.publish(self.Follow_Twist)
    
    def Openmv_Data_Transmit(self, dir):
        #self.ser_OPMV.write(dir)
        1

class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.clock()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.clock()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D

# if __name__ == '__main__':
#     follow_line = FollowLine()   #init class FollowLine
#     ROS_Ctrl = ROSCtrl()         #init class ROSCtrl
#     RotateRobot()
#     if follow_line.ser_OPMV.is_open:
#         print("OPmv module serial port --/dev/ttyUSB1 open successfully!\r\n")
#         rate = rospy.Rate(100)
#         while not rospy.is_shutdown() and follow_line.ser_OPMV.is_open:
#             if not ROS_Ctrl.Cancel_Motion:
#                 follow_line.Openmv_Data_Receive()
#             else:
#                 with LimitedRun('Send_Stop_msg', 1) as limited_run:
#                     if limited_run:
#                         follow_line.Move_Stop()
#                         rospy.loginfo("Move_Stop!")
#             if ROS_Ctrl.Direction_index != ROS_Ctrl.Direction_index_Last:
#                 follow_line.Openmv_Data_Transmit(ROS_Ctrl.Direction_index)
#                 ROS_Ctrl.Direction_index_Last = ROS_Ctrl.Direction_index
#             rate.sleep()
#         rospy.loginfo("Node exit!")
#     else: rospy.loginfo("Serial open Failed!")

if __name__ == '__main__':
    rospy.init_node("FollowLine", anonymous=False)
    ROS_Ctrl = ROSCtrl()         #init class ROSCtrl
    Rotate_robo = RotateRobot()
    print("Init successfully!\r\n")
    Finish_Rotate_flag = False
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if ROS_Ctrl.Rotate_Motion:
            if not Finish_Rotate_flag:
                Finish_Rotate_flag = Rotate_robo.Robot_Rotate(1, 180)
            else:
                ROS_Ctrl.Rotate_Motion = 0
                Finish_Rotate_flag = False
        rate.sleep()
    rospy.loginfo("Node exit!")
