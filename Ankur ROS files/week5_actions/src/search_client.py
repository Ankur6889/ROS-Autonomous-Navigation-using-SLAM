#!/usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction,SearchFeedback, SearchActionGoal

class preemptiveActionClient():
   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.current_distance = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance reached: {feedback_data.current_distance_travelled:.1f} m...")
        #if self.current_distance>=20:
        #    rospy.logwarn("Received a shutdown request. Cancelling Goal...")
        #    self.client.cancel_goal()
        #    rospy.logwarn("Goal Cancelled")
        #    print(f"{self.current_distance} meters reached")

    def __init__(self):
        self.current_distance = 0
        self.action_complete = False

        node_name = "preemptive_move_robot_action_client"
        action_server_name = "/move_robot_action_server"
        
        rospy.init_node(node_name)

        self.rate = rospy.Rate(1)

        self.goal = SearchActionGoal()

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            print(f"RESULT: {self.current_distance} meters reaches.")

    def send_goal(self, speed, distance):
        self.goal.goal.fwd_velocity = speed
        self.goal.goal.approach_distance = distance
        
        # send the goal to the action server:
        self.client.send_goal(self.goal.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(speed=0.15,  distance= 0.4)
        print("Robot exploring environment...")
        while self.client.get_state() < 2:
            print(f"STATE: Current state code is {self.client.get_state()}")
            self.rate.sleep()
        self.action_complete = True
        print(f"RESULT: Action State = {self.client.get_state()}")
        outcome=self.client.get_result()
        print(f"RESULT: {outcome}")

if __name__ == '__main__':
    move_Robot = preemptiveActionClient()
    move_Robot.main()