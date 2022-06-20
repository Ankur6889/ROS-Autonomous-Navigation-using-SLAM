#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
class Subscriber():

    def callback(self, topic_data):
        orientation_x = topic_data.pose.pose.orientation.x
        orientation_y = topic_data.pose.pose.orientation.y
        orientation_z = topic_data.pose.pose.orientation.z
        orientation_w = topic_data.pose.pose.orientation.w

        position_x = topic_data.pose.pose.position.x
        position_y = topic_data.pose.pose.position.y
        position_z = topic_data.pose.pose.position.z
        (roll, pitch, yaw)=euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')
        print(f"x = '{position_x}', y = '{position_y}', theta_z = '{position_z}'")
        #print(f"x = '{roll}', y = '{pitch}', theta_z = '{yaw}'")

    def __init__(self):
        self.node_name = "odom_subscriber"
        topic_name = "odom"

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, Odometry, self.callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()