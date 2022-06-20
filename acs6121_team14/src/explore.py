#!/usr/bin/env python3
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import enum
import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import time
from tf.transformations import euler_from_quaternion
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import threading
import time
from matplotlib.colors import Normalize
import matplotlib.cm
import datetime
import random
import subprocess
import math
import roslaunch

class Explore:

    def __init__(self):
        """ Initialize environment
        """
        bringup_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(bringup_uuid)
        self.bringup_launch=roslaunch.parent.ROSLaunchParent(bringup_uuid,["/home/student/catkin_ws/src/acs6121_team14/launch/bringup.launch"])
        self.bringup_launch.start()
        rospy.loginfo("start simulators")

        node_name="explore"
        rospy.init_node(node_name,log_level=rospy.DEBUG)
        self.start_time=time.perf_counter()
        self.goal_set_time=time.perf_counter()
        self.time_out=10
        self.end_time=30
        self.isEnd=False
        self.map_saved=False
    
        # Initialize rate:
        self.rate = rospy.Rate(0.5)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        rospy.logdebug("move_base is ready") 

        self.x = 0
        self.y = 0
        self.robot_x=0
        self.robot_y=0
        self.robot_theta_z=0
        self.completion = True
        # part to visit: idn to visit, if visited, -1
        self.visitFlag=[12,17,16,11,6,7,8,13,18,23,22,21,20,15,10,5,0,1,2,3,4,9,14,19,24]
        # generate 5 times before move to next point
        self.itr_n=5

        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.count = 0
        rospy.on_shutdown(self.shutdown_ops)
        while not rospy.is_shutdown():
            # wait for 0.1 seconds (10 HZ) and publish again
            self.rate.sleep()
    
    def shutdown_ops(self):
        
        map_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(map_uuid)
        map_launch=roslaunch.parent.ROSLaunchParent(map_uuid,["/home/student/catkin_ws/src/acs6121_team14/launch/saving.launch"])
        map_launch.start()
        rospy.loginfo("start saving map")
        rospy.sleep(3)
        map_launch.shutdown()
        rospy.logwarn("Received a shutdown request. Cancelling Goal...")
        self.move_base.cancel_goal()
        rospy.logwarn("Goal Cancelled")
        # stop robot
        # self.cmd_vel.publish(Twist())
        quit_time=time.perf_counter()-self.start_time
        print(f"quit at: {quit_time} s. ")
        unvisited=[v for v in self.visitFlag if v!=-1]
        print(f"cells unvisited: {unvisited}")
        # save map
        """cmd=["rosrun", "map_server","map_saver","-f", "/home/student/catkin_ws/src/slam_nav/maps/emap_exploration"]
        proc=subprocess.Popen(cmd)
        self.rate.sleep()
        proc.terminate()"""
        self.isEnd=True
        self.bringup_launch.end()
                
    def cellToId(self, row, col):
        idx=round(row*384+col)
        return idx
    
    def posToId(self, x, y):
        col=round((x+10)/0.05)
        row=round((y+10)/0.05) # x=column * resolution + origin_x
        return self.cellToId(row, col)
    
    def numToRandPos(self, n):
        tolerance=0.25
        size=1.1
        if n<5 and n>0:
            lower=-1.65+size*(n-1)
            upper=-1.65+size*n
            if n==1:
                lower=lower+tolerance
            elif n==3:
                upper=upper+tolerance
            
            res=random.gauss((lower+upper)/2, (upper-lower)/4) #95% inside cell
            if res<lower:
                res=lower
            elif res>upper:
                res=upper
            return res
        else:
            print(f"out of cell range")
        return 0
    
    def idnToP(self, idn):# idn should be 0~8
        row=idn/5 # 0,1,2,3
        col=idn%5
        x=self.numToRandPos(col+1)
        y=self.numToRandPos(row+1)
        return self.posToId(x, y)

    def posToIdn(self, x, y):
        """row_n=n/3
        col_n=n%3
        self.x=self.numToRandPos(col_n+1)
        self.y=self.numToRandPos(row_n+1)"""
        row=math.floor((y+2.75)/1.1)
        col=math.floor((x+2.75)/1.1)
        idn=round(row*5+col)
        print(f" transfer ({self.x, self.y}) to {idn} at row {row}, col {col}")
        return idn
        
    def isInIdn(self, x,y):
        size=1.1
        row=math.floor((y+2.75)/1.1)
        col=math.floor((x+2.75)/1.1)
        lower_x=-1.65+size*row
        lower_y=-1.65+size*col
        tolerance=0.2
        if x>lower_x+tolerance and x<lower_x+size-tolerance and y>lower_y+tolerance and y<lower_y+size-tolerance:
            return round(row*5+col)
        else:
            return -1

    def odom_callback(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can 
        # access them elsewhere within our Square() class):
        self.robot_x = pos_x
        self.robot_y = pos_y
        self.robot_theta_z = yaw 

        # check if in the cells
        if (time.perf_counter()-self.start_time)%5==0:
            idn=self.isInIdn(self.robot_x, self.robot_y)
            if idn!=-1:
                for i, v in enumerate(self.visitFlag):
                    if v==idn:
                        self.visitFlag[i]=-1
                        print(f" set {idn} to {self.visitFlag[i]}")
                rospy.logdebug(f"cell {idn} visited")
        
    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        # find free idx 4500
        """candidates=[i for i,v in enumerate(data.data) if v!=-1 and v<0.2]
        max_idx=max(candidates)
        min_idx=min(candidates)"""
        if not self.isEnd:
            if self.completion: # availiable for sending
                self.completion =False

                valid = False
                idRobot=self.posToIdn(self.robot_x, self.robot_y)
                while valid is False: # resample till valid
                    # idp=randrange(len(candidates))
                    """row = randrange(157, 227)
                    col = randrange(157, 227)
                    px=random.uniform(-1.5, 1.5)
                    py=random.uniform(-1.5, 1.5)"""
                    # idp = randrange(len(data.data)) # (min, max)
                    candidates=[i for i in self.visitFlag if i!=-1]
                    print(f"unvisited cell: {candidates}")
                    if not candidates: # shut_down if empty candidates
                        self.shutdown_ops()
                        break
                    for n in candidates:
                        # generate a random point in block n: 0~8
                        row_n=math.floor(n/5)
                        col_n=n%5
                        itr=0
                        self.x=self.numToRandPos(col_n+1)
                        self.y=self.numToRandPos(row_n+1)
                        idp=self.posToId(self.x, self.y)
                        self.map = data.data[idp]
                        # check whether it is valid
                        if self.map!=-1 and self.map<0.2:
                            valid = self.check_neighbors(data, idp)
                        if valid:
                            idn=n
                            break

                """row = idp / data.info.width
                col = idp % data.info.width

                self.x = col * data.info.resolution +data.info.origin.position.x  # column * resolution + origin_x
                self.y = row * data.info.resolution + data.info.origin.position.y  # row * resolution + origin_x
                idn=self.posToIdn(self.x, self.y)"""
                
                rospy.logdebug(f"set goal in cell {idn} when robot at cell {idRobot}")
                # Start the robot moving toward the goal
                self.goal_set_time=time.perf_counter()
                self.set_goal()
            
            # wait some time for goal to complete, if stack move to next goal
            if time.perf_counter()-self.goal_set_time>self.time_out:
                idn=self.posToIdn(self.x, self.y)
                rospy.logdebug(f"skip goal: {self.x, self.y} at cell {idn}")
                self.completion = True
            
            """if time.perf_counter()-self.start_time>self.end_time and not self.map_saved:
                rospy.logdebug(f"time is up, save map now")
                self.map_saved=True
                self.shutdown_ops()
                self.rate.sleep()"""

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
        self.move_base.send_goal(goal, self.feedback_callback)
        
    def feedback_callback(self, status, result):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        self.completion = True #move to next sample

        # Goal reached
        if status == 3:
            idn=self.posToIdn(self.x, self.y)
            rospy.loginfo("Goal succeeded ")
        else:
            idn=self.posToIdn(self.robot_x, self.robot_y)
        
        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal aborted")

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal rejected")
        
        if idn!=-1:
            for i, v in enumerate(self.visitFlag):
                if v==idn:
                    self.visitFlag[i]=-1
                    print(f" set {idn} to {self.visitFlag[i]}")
            rospy.logdebug(f"cell {idn} visited")

    def check_neighbors(self, data, idn):
        """ Checks neighbors for random points on the map.
        """
        distance=(self.x-self.robot_x)+(self.y-self.robot_y) # <1.5
        rotation=abs(math.atan((self.y-self.robot_y)/(self.x-self.robot_x))) # <math.pi
        if distance>1.5 or rotation>math.pi/2:
            return False
        unknown=0

        for row in range(-6, 6): # 10x10 cells 0.05*10=0.5m free rotation
            for col in range(-6, 6):
                n_cell= row * 384 + col
                try:
                    if data.data[idn + n_cell] == -1:
                        unknown+=1
                    elif data.data[idn+n_cell]>0.2:
                        return False
                except IndexError:
                    pass
        if unknown<1:
            return True
        else:
            return False

    def main_loop(self):
        if not self.isEnd:
            rospy.spin()

if __name__ == '__main__':
    explore=Explore()
    try:
        explore.main_loop()
    except:
        rospy.ROSInterruptException