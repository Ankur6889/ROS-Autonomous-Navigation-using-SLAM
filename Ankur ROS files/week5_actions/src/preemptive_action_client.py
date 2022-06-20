#!/usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback

class preemptiveActionClient():
   
    def feedback_callback(self, feedback_data: CameraSweepFeedback):
        self.captured_images = feedback_data.current_image
        print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
            f"Image(s) captured so far: {self.captured_images}...")
        if self.captured_images>=5:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            print(f"{self.captured_images} images captured")

    def __init__(self):
        self.captured_images = 0
        self.action_complete = False

        node_name = "preemptive_camera_sweep_action_client"
        action_server_name = "/camera_sweep_action_server"
        
        rospy.init_node(node_name)

        self.rate = rospy.Rate(1)

        self.goal = CameraSweepGoal()

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    CameraSweepAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            print(f"RESULT: {self.captured_images} image(s) saved.")

    def send_goal(self, images, angle):
        self.goal.sweep_angle = angle
        self.goal.image_count = images
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(images = 10, angle = 90)
        i = 1
        print("While we're waiting, let's do our seven-times tables...")
        while self.client.get_state() < 2:
            print(f"STATE: Current state code is {self.client.get_state()}")
            print(f"TIMES TABLES: {i} times 7 is {i*7}")
            i += 1
            self.rate.sleep()
        self.action_complete = True
        print(f"RESULT: Action State = {self.client.get_state()}")
        print(f"RESULT: {self.captured_images} images saved to {self.client.get_result()}")

if __name__ == '__main__':
    move_Robot = preemptiveActionClient()
    move_Robot.main()