#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from com2009_msgs.srv import ApproachResponse,Approach
import numpy as np



pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()

class Avoider():
    def callback_function(self,service_request):

        service_response = ApproachResponse()

        print(f"The '{self.service_name}' Server received a 'forward' request and the robot will now move forward ...")
        StartTime = rospy.get_rostime()

        vel.linear.x = service_request.approach_velocity
        pub.publish(vel)
        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (self.min_distance) > service_request.approach_distance:
            continue

        vel.linear.x = 0.0
        pub.publish(vel)

        service_response.response_message = f"Object ahead moving forward have elapsed, stopping the robot..."
        return service_response

    def scan_callback(self, scan_data):
        # From the front of the robot, obtain a 20 degree 
        # arc of scan data either side of the x-axis 
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        # combine the "left_arc" and "right_arc" data arrays, flip them so that 
        # the data is arranged from left (-20 degrees) to right (+20 degrees)
        # then convert to a numpy array
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        # Obtain the minimum distance measurement within the "front_arc" array
        # to indicate when we are getting close to something up ahead:
        self.min_distance = front_arc.min()

        # Advanced:
        # Create another numpy array which represents the angles 
        # (in degrees) associated with each of the data-points in 
        # the "front_arc" array above:
        arc_angles = np.arange(-20, 21)
        # determine the angle at which the minimum distance value is located
        # in front of the robot:
        self.object_angle = arc_angles[np.argmin(front_arc)]
    
    def __init__(self):
        self.node_name="Lase_Sub"
        self.service_name = "move_service"
        rospy.init_node(f"{self.service_name}_server")
        topic_name = "/scan"
        self.sub = rospy.Subscriber(topic_name, LaserScan, self.scan_callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")
    
    def main_loop(self):
        my_service = rospy.Service(self.service_name, Approach, self.callback_function)
        rospy.loginfo(f"the '{self.service_name}' Server is ready to be called...")
        rospy.spin()


if __name__ == '__main__':
    move_Robot = Avoider()
    move_Robot.main_loop()
    try:
        move_Robot.main_loop()
    except rospy.ROSInterruptException:
        pass
