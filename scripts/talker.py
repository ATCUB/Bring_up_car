import rospy
from nav_msgs.msg import Odometry

rospy.init_node('odom_listener')

def odom_callback(msg):
    position = msg.pose.pose.position
    print('Current position: x={}, y={}, z={}'.format(position.x, position.y, position.z))

odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
new_odom_msg = Odometry()
new_odom_msg.pose.pose.position.x = 1.0
new_odom_msg.pose.pose.position.y = 2.0
new_odom_msg.pose.pose.position.z = 3.0
odom_pub.publish(new_odom_msg)

rospy.spin()
