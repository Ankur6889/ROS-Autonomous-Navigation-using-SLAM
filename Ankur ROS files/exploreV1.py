#!/usr/bin/env python3
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import threading
import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import threading
import time
import matplotlib.image
from matplotlib.colors import Normalize
import matplotlib.cm
import datetime
from random import randrange

class Explore:

    def __init__(self):
        """ Initialize environment
        """
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 

        self.x = 0
        self.y = 0
        self.completion = 0
        self.action_complete=False

        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.count = 0
        time.sleep(5)
        #rospy.on_shutdown(self.shutdown_ops)
    
    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
            print(f"RESULT: {self.count} points travelled.")



    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        # find free idx
        candidates=[i for i,v in enumerate(data.data) if v!=-1 and v<0.2]
        print(f"number of free cells: {len(candidates)}.")
        valid = False
        while valid is False: # resample till valid
            idp=randrange(len(candidates))
            # map_size = randrange(len(data.data)) # (min, max)
            # self.map = data.data[map_size]
            self.map = data.data[idp]

            edges = self.check_neighbors(data, idp)
            if edges is True:
                valid = True
            
        row = idp / 384
        col = idp % 384

        self.x = col * 0.05 - 10  # column * resolution + origin_x
        self.y = row * 0.05 - 10  # row * resolution + origin_x
        
        if self.completion % 2 == 0:
            self.completion += 1
            # Start the robot moving toward the goal
            self.set_goal()
    

    def set_goal(self):
        """ Set goal position for move_base.
        """
        rospy.logdebug("Setting goal")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.logdebug(f"goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}")
        self.move_base.send_goal(goal, self.goal_status)
        


    def goal_status(self, status, result):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        self.completion += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal succeeded")

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal aborted")

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal rejected")


    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map.
        """
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * 384 + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False
    




def main():
    """ The main() function """
    rospy.init_node('explore', log_level=rospy.DEBUG)
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
