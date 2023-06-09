#!/usr/bin/env python2
# encoding: utf-8
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
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
# from dynamic_reconfigure.client import Client
from yahboomcar_bringup.cfg import FollowPIDConfig
import matplotlib.pyplot as plt
from typing import Any

RAD2DEG = 180 / math.pi

#limit excute num
class LimitedRun(object):
    run_dict = {}

    def __init__(self,
                 tag: Any = 'default',
                 limit: int = 1):
        self.tag = tag
        self.limit = limit
    
    def reset(self, tag:Any):
        if tag in self.run_dict.keys():
            self.run_dict[tag] = 0

    def __enter__(self):
        if self.tag in LimitedRun.run_dict.keys():
            LimitedRun.run_dict[self.tag] += 1
        else:
            LimitedRun.run_dict[self.tag] = 1
        return LimitedRun.run_dict[self.tag] <= self.limit

    def __exit__(self, exc_type, exc_value, traceback):
        return


#class controller include of sub_joy and  pub_cmdVel
class ROSCtrl:
    def __init__(self):
        self.Cancel_Motion = 0
        self.Direction_index = 4
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
            左摇杆(左正右负): axes[0]
            左摇杆(上正下负): axes[1]
            右摇杆(左正右负): axes[2]
            右摇杆(上正下负): axes[3]
            R2(按负抬正): axes[4] cancel_nav
            L2(按负抬正): axes[5]
            左按键(左正右负): axes[6]
            左按键(上正下负): axes[7]
            buttons 15:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            A: buttons[0]
            B: buttons[1]
            X: buttons[3]
            Y: buttons[4]
            L1: buttons[6]
            R1: buttons[7] RGBLight
            SELECT: buttons[10]
            START: buttons[11] Buzze
            左摇杆按下: buttons[13] linear Gear control
            右摇杆按下: buttons[14] angular Gear control
        '''
        if joy_data.buttons[0] == 1:
            self.Cancel_Motion ^= 1

        if joy_data.buttons[1] == 1:
            self.Direction_index += 1
            if self.Direction_index > 4:
                self.Direction_index = 0
            rospy.loginfo("Now Direction is %s", self.Direction[self.Direction_index])

    def cancel(self):
        self.sub_Joy.unregister()
        self.pub_cmdVel.unregister()

class FollowLine:
    def __init__(self):
        rospy.init_node("FollowLine", anonymous=False)
        self.scale = 2000
        self.dyn_update = False
        self.Start_state = True
        self.Data_Of_OPENMV = 0
        self.FollowPID = (60, 0, 20)
        self.linear_x = 0.4
        self.linear_y = 0
        self.angular_z = 0
        self.error_of_z = 0
        self.flip = True
        self.Track_state = 'waiting'
        self.Buzzer_state = False
        self.Send_Count = 0
        Server(FollowPIDConfig, self.dynamic_reconfigure_callback)
        # self.dyn_client = Client("FollowLine", timeout=60)
        self.PID_init()
        self.ros_ctrl = ROSCtrl()
        self.Follow_Twist = Twist()
        self.pub_Buzzer = rospy.Publisher("/Buzzer", Bool, queue_size=1)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_IR = rospy.Subscriber("/serial_IRdata_msg", Int16, queue_size=1)
        self.sub_vel = rospy.Subscriber("/pub_vel", Twist, queue_size=1)
        self.ser_OPMV = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

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
            [self.FollowPID[0] / 1.0 / (self.scale), 0],
            [self.FollowPID[1] / 1.0 / (self.scale), 0],
            [self.FollowPID[2] / 1.0 / (self.scale), 0])
        
    def dynamic_reconfigure_callback(self, config, level):
        self.scale = config['scale']
        self.linear_x = config['linear_x']
        self.FollowPID = (config['Kp'], config['Ki'], config['Kd'])
        self.PID_init()
        return config

    def Openmv_Data_Receive(self):
        self.Openmv_Data_Rx = self.ser_OPMV.readline()
        if self.Openmv_Data_Rx:
            Openmv_Data_Rx = Openmv_Data_Rx.decode("utf-8")
            if Openmv_Data_Rx[0] == 's' or self.Track_state == 'waiting':
                self.Move_Stop()
            else:
                Openmv_Data_Rx = Openmv_Data_Rx.split(",")
                self.linear_y = float(Openmv_Data_Rx[0])
                self.angular_z = float(Openmv_Data_Rx[1])
                self.error_of_z = float(Openmv_Data_Rx[2])

    def Move_Stop(self):
        self.Follow_Twist.angular.x = 0
        self.Follow_Twist.angular.y = 0
        self.Follow_Twist.angular.z = 0
        if self.Send_Count:
            self.pub_vel.publish(self.Follow_Twist)
            self.Send_Count = 0;    
    def Openmv_Data_Transmit(self, dir):
        self.ser_OPMV.write(dir)

    def process(self, IR_data):
        self.Track_state = 'tracking'
        if self.Track_state == 'tracking':
            threading.Thread(target=self.execute, args=(IR_data, )).start()
        else:
            if self.Start_state == True:
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.Start_state = False

    def execute(self, IRdata):
        self.Start_state = True
        twist =Twist()
        b = Bool()
        if IRdata == 300:
            self.ros_ctrl.pub_cmdVel.publish(Twist())
            rospy.loginfo("something wrong happened!!!Or maybe a cross ahead.")
        else:
            [z_PID, _] = self.PID_controller.update([int(IRdata), 0])
            if self.flip == True:twist.angular.z = -z_PID
            else: twist.angular.z = +z_PID
            twist.linear.x = self.linear_x
            if self.Buzzer_state == True:
                b.data = False
                for i in range(3): self.pub_Buzzer.publish(b)
                self.Buzzer_state = False
            self.ros_ctrl.pub_cmdVel.publish(twist)
            print("Publish successfully!\r\n")


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

if __name__ == '__main__':
    follow_line = FollowLine()   #init class FollowLine
    ROS_Ctrl = ROSCtrl()         #init class ROSCtrl
    ser_OPmv = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    if ser_OPmv.is_open:
        print("OPmv module serial port --/dev/ttyUSB0 open successfully!\r\n")
        rate = rospy.Rate(100)
        while ser_OPmv.is_open:
            if not ROS_Ctrl.Cancel_Motion:
                follow_line.Openmv_Data_Receive()
                LimitedRun.reset('Send_Stop_msg')
            else:
                with LimitedRun('Send_Stop_msg', 1) as limited_run:
                    if limited_run:
                        follow_line.Move_Stop()
            if ROS_Ctrl.Direction_index != ROS_Ctrl.Direction_index_Last:
                follow_line.Openmv_Data_Transmit(ROS_Ctrl.Direction_index)
                ROS_Ctrl.Direction_index_Last = ROS_Ctrl.Direction_index
            rate.sleep()
    else: print("Serial open Failed!\r\n")
