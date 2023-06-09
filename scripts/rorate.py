#!/usr/bin/env python
# encoding: utf-8
#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
class RotateRobot:
    def __init__(self):
        rospy.init_node('rotate_robot')
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(50)
        self.current_yaw = 0.0
        self.target_yaw = 180.0
        self.init = 0
        self.beginyaw = 0

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)
        if self.init == 0:
            self.beginyaw = self.current_yaw
            self.init = 1
    def quaternion_to_yaw(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def run(self):
        while not rospy.is_shutdown():
            if abs(self.current_yaw- self.beginyaw- self.target_yaw) > 5.0:
                self.twist.angular.z = 1
            else:
                self.twist.angular.z = 0.0
                rospy.set_param('enable',0)
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
if 1:
#if rospy.get_param('enable'):
    try:
        robot = RotateRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
else:
    1
