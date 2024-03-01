#!/usr/bin/env python3
"""
 The explore force robot wonder in enviroment to random setted point
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
from random import randrange
import time

class Explore:

    def __init__(self):

        self.Rate = rospy.Rate(1)
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("MB ready!") 
        self.X = 0
        self.Y = 0
        self.Completion = 0

        # Initialize subscribers:
        self.Map = OccupancyGrid()
        self.SubMap = rospy.Subscriber('/map', OccupancyGrid, self.MapCallback)
        self.Count = 0
        time.sleep(8)


    def MapCallback(self, data):
        bIsValid = False
        while bIsValid is False:
            MapSize = randrange(len(data.data))
            self.Map = data.data[map_size]
            _edges = self.CheckNeighbors(data, map_size)
            if self.Map != -1 and self.map <= 0.2 and edges is True:
                bIsValid = True
            
        _mRow = map_size / 384
        _mCol = map_size % 384

        self.X = mCol * 0.05 - 10  # column * resolution + origin_x
        self.Y = mRow * 0.05 - 10  # row * resolution + origin_x
        
        if self.Completion % 2 == 0:
            self.Completion += 1
            # Start the robot moving toward the goal
            self.SetGoal()
    

    def SetGoal(self):
        _goal = MoveBaseGoal()

        _goal.target_pose.header.frame_id = "map"
        _goal.target_pose.header.stamp = rospy.Time.now()
        _goal.target_pose.pose.position.x = self.x
        _goal.target_pose.pose.position.y = self.y
        _goal.target_pose.pose.orientation.w = 1.0
        self.move_base.send_goal(_goal, self.GoalStatus)


    def GoalStatus(self, status, result):
        self.Completion += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal succeeded")

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal aborted")

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal rejected")


    def CheckNeighbors(self, data, map_size):

        _unknowns = 0
        _obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                mRow = x * 384 + y
                try:
                    if data.data[map_size + mRow] == -1:
                        _unknowns += 1
                    elif data.data[map_size + mRow] > 0.65:
                        _obstacles += 1
                except IndexError:
                    pass
        if _unknowns > 0 and _obstacles < 2:
            return True
        else:
            return False


def main():
    rospy.init_node('explore', log_level=rospy.DEBUG)
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
