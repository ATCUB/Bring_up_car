#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import threading
import math
import rospkg
import rospy
from std_msgs.msg import String, Float32, Int32, Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Define the maze size
MAZE_WIDTH = 4  # in meters
MAZE_HEIGHT = 4  # in meters

# Define the maze resolution
MAZE_RES = 0.4  # in meters

# Define the initial position of the robot in the maze
robot_x = 0.0  # in meters
robot_y = 0.0  # in meters

# Define the initial orientation of the robot in the maze
robot_theta = 0.0  # in radians

# Define the callback function to process the odometry data
def odom_callback(msg):
    global robot_x, robot_y, robot_theta

    # Get the position of the robot from the odometry message
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y

    # Get the orientation of the robot from the odometry message
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # Convert the yaw angle from radians to degrees
    robot_theta = yaw * 180.0 / 3.14159265358979323846

# Initialize the ROS node
rospy.init_node('maze_location')

# Subscribe to the odometry topic
rospy.Subscriber('/odom', Odometry, odom_callback)

# Loop until the node is shut down
while not rospy.is_shutdown():
    # Compute the grid position of the robot
    grid_x = int(robot_x / MAZE_RES)
    grid_y = int(robot_y / MAZE_RES)

    # Print the current grid position of the robot
    rospy.loginfo('Robot is at grid position (%d, %d)' % (grid_x, grid_y))
    rospy.loginfo('Robot is heading  %d)' ,robot_theta)

    # Sleep for a short period of time
    rospy.sleep(0.1)

