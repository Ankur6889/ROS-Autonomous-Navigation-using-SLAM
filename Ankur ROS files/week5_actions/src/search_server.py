#!/usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchAction, SearchActionFeedback, SearchActionResult

# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import radians, sqrt
import datetime as dt
import os

class CameraSweepAS(object):
    feedback = SearchActionFeedback() 
    result = SearchActionResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/move_robot_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()


        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
    
    
    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity >= 0.26 or goal.fwd_velocity<=0:
            print("Invalid speed.  Select a value between 0 and 0.26.")
            success = False
        if goal.approach_distance >0.5 or goal.approach_distance<0.1:
            print("Invalid distance. Select a value between 0.1 and 0.5")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return



        print(f"\n#####\n"
            f"The 'move_robot_action_server' has been called.\n"
            f"Goal: Robot moving at speed: {goal.fwd_velocity } m/s and will stop in front of the obstacle in distance: {goal.approach_distance} m...\n\n"
            f"Commencing the action...\n"
            f"#####\n")
        
        # set the robot velocity:
        self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
        
        # Get the current robot odometry:
        ref_posx = self.robot_odom.posx
        ref_posy = self.robot_odom.posy

        laser_data_min= self.robot_scan.min_distance
        self.feedback.feedback.current_distance_travelled=0
        while laser_data_min > goal.approach_distance:
            self.robot_controller.publish()
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the moving robot.")
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            
                
            # populate the feedback message and publish it:
            self.feedback.feedback.current_distance_travelled = self.feedback.feedback.current_distance_travelled + sqrt(pow(ref_posx-self.robot_odom.posx,2) + pow(ref_posy-self.robot_odom.posy,2))
            self.actionserver.publish_feedback(self.feedback.feedback)

            # update the reference odometry:
            ref_posx = self.robot_odom.posx
            ref_posy = self.robot_odom.posy
            laser_data_min= self.robot_scan.min_distance
        
        if success:
            self.result.result.closest_object_angle=self.robot_scan.closest_object_position
            self.result.result.total_distance_travelled=self.feedback.feedback.current_distance_travelled
            self.result.result.closest_object_distance=self.robot_scan.min_distance
            rospy.loginfo("Moving Robot completed sucessfully.")
            self.actionserver.set_succeeded(self.result.result)
            self.robot_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("camera_sweep_action_server")
    CameraSweepAS()
    rospy.spin()
