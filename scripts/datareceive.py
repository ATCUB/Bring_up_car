#!/usr/bin/env python
# encoding: utf-8
import struct
import serial
import rospy
import time
import gzip
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server


rospy.init_node('op',anonymous=True)
rate = rospy.Rate(500)
vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
twist=Twist()
ser = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=0.5)
if ser.is_open :
    print("open")
else:
    print("error")



#if __name__=="__main__":

while ser.is_open:
    vel = ser.readline()
    if vel:
        vel = vel.decode("utf-8")
        if vel [0]== "s":
            twist.angular.z = 0
            twist.linear.y = 0
            twist.linear.x = 0
            vel_pub.publish(twist)
	    print(twist)
        else:#读一行数据
	    print(vel)
            a = vel.split(",")
            y = float(a[0])
	    y = -y
	    #y = 0
            z = float(a[1])
	    z = -z
            x = 0.2
            if y>100:
                y=100
            twist.angular.z = z/100
            twist.linear.y = y/1000
	    if abs(y)< 0.3:
	        y = 0
            twist.linear.x = x
            vel_pub.publish(twist)
            print(twist)

        rate.sleep()
        

