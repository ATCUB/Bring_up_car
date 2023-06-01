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
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
# from dynamic_reconfigure.client import Client
from yahboomcar_bringup.cfg import FollowPIDConfig
import matplotlib.pyplot as plt
RAD2DEG = 180 / math.pi

#class controller include of sub_joy and  pub_cmd_vel
class ROSCtrl:
    def __init__(self):
        self.Joy_active = False
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_JoyState = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())

    def cancel(self):
        self.sub_JoyState.unregister()
        self.pub_cmdVel.unregister()

class FollowLine:
    def __init__(self):
        rospy.init_node("FollowLine", anonymous=False)
        self.scale = 2000
        self.dyn_update = False
        self.Start_state = True
        self.FollowPID = (60, 0, 20)
        self.linear = 0.4
        self.flip = True
        self.Track_state = 'waiting'
        self.Buzzer_state = False
        Server(FollowPIDConfig, self.dynamic_reconfigure_callback)
        # self.dyn_client = Client("FollowLine", timeout=60)
        self.PID_init()
        self.ros_ctrl = ROSCtrl()
        self.pub_Buzzer = rospy.Publisher("/Buzzer", Bool, queue_size=1)
        self.sub_IR = rospy.Subscriber("/serial_IRdata_msg", Int16, queue_size=1)
        self.sub_vel = rospy.Subscriber("/pub_vel", Twist, queue_size=1)
        self.ser_IR = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

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
        self.linear = config['linear']
        self.FollowPID = (config['Kp'], config['Ki'], config['Kd'])
        self.PID_init()
        return config

    def process(self, IR_data):
        self.Track_state = 'tracking'
        #elif presskey == ord('w') or presskey == 119:self.Track_state = 'waiting'
        #elif presskey == ord('r') or presskey == 114:self.Reset()
        #elif presskey == ord('q') or presskey == 114: self.cancel()
#        if self.Track_state == 'waiting':
        if self.Track_state == 'tracking':
            threading.Thread(target=self.execute, args=(IR_data, )).start()
        else:
            if self.Start_state == True:
                self.ros_ctrl.pub_cmdVel.publish(Twist())
                self.Start_state = False

    def execute(self, IRdata):
        # if self.ros_ctrl.Joy_active == True:
        #     if self.Start_state == True:
        #         self.PID_init()
        #         self.Start_state = False
        #     return
        self.Start_state = True
        twist =Twist()
        b = Bool()
        if IRdata == 300:
            self.ros_ctrl.pub_cmdVel.publish(Twist())
            rospy.loginfo("something wrong happened!!!Or maybe a cross ahead.")
            #self.Buzzer_state = True
            #b.data = True
            #self.pub_Buzzer.publish(b)
        else:
            [z_PID, _] = self.PID_controller.update([int(IRdata), 0])
            if self.flip == True:twist.angular.z = -z_PID
            else: twist.angular.z = +z_PID
            twist.linear.x = self.linear
            if self.Buzzer_state == True:
                b.data = False
                for i in range(3): self.pub_Buzzer.publish(b)
                self.Buzzer_state = False
            self.ros_ctrl.pub_cmdVel.publish(twist)
            print("publish successfully!\r\n")


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
    # IR_array = [] 
    follow_line = FollowLine()   #init class FollowLine
    ser_IR = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
    if ser_IR.is_open:
        print("IR module serial port --/dev/ttyUSB0 open successfully!\r\n")
        rate = rospy.Rate(100)
        while ser_IR.is_open:
            IR_msg = rospy.wait_for_message('/serial_IRdata_msg', Int16, timeout = None)
            # IR_array.append(IR_msg.data/100.0)
            follow_line.process(IR_msg.data / 100.0)
            # if  len(IR_array) >= 20:
            #         plt.ion()
            #         plt.clf()
            #         plt.axis([0,len(IR_array)+1,-30,30])
            #         plt.plot(range(len(IR_array)),IR_array)
            #         plt.pause(0.001)
            #         plt.ioff()
	    #print("I'm here!\r\n")
	    #print(IR_msg)
            rate.sleep()
    else: print("serial open Failed!\r\n")
