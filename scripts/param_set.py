#!/usr/bin/env python
# encoding: utf-8
import rospy
rospy.set_param("enable",0)
flag = rospy.get_param("enable")
def timer_callback(event):
    global flag
    if rospy.get_param("enable") == 0:
         rospy.set_param("enable",1)
    else:
   	 rospy.set_param("enable",0)
   # rospy.set_param("enable",1)
    flag = rospy.get_param("enable")
if __name__ == '__main__':
    rospy.init_node('flag_node')
    timer = rospy.Timer(rospy.Duration(2), timer_callback)
    while not rospy.is_shutdown():
        # do something with flag
        print("flag = ", flag)

