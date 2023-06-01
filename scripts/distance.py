#!/usr/bin/env python
# encoding: utf-8
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
x = 0
y = 0
new_x = 0
new_y = 0
#rospy.init_node("odom_listener")

#def  odom_callback(msg):
rospy.init_node('position_diff')
rate = rospy.Rate(1)
def callback(msg):
    # 获取当前odom消息中的position
    global x
    global y
    global new_x
    global new_y
    position = msg.pose.pose.position
    new_x = position.x
    new_y = position.y
    diff_x = x - new_x
    diff_y = y - new_y
    print('Position difference after 3 seconds: ({}, {})'.format(diff_x, diff_y))
    x = new_x
    y = new_y
while 1:	
	rospy.Subscriber('/odom', Odometry, callback)
        rate.sleep()
	

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
