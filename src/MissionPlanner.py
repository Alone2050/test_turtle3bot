#!/usr/bin/python

import random
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty
from ObstacleController import ObstacleController

class MissionPlanner:

    def __init__(self):

        self.MissionPoints = [(1.75, 1.0), (2.0, 0.0), (1.75, -1.0),
                               (1.0, -1.75), (0.0, -2.0), (-1.0, -1.75),
                               (-1.75, -1.0), (-2.0, 0.0), (-1.75, 1.0),
                               (-1.0, 1.75), (0.0, 2.0), (1.0, 1.75),
                               (-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5),
                               (0.5, -0.5)]

        self.GoalCoords = ()
        self.PreviusGoalCoords = ()

        self.PoseInitPub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        while self.pose_init_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect")
            rospy.sleep(1)

        self.GoalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        while self.GoalPub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect")
            rospy.sleep(1)

        self.MissionStatusSub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.__mission_status_monitor)

        rospy.wait_for_service('/move_base/clear_costmaps')
        self.ClearCostMaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0

        self.PoseInitPub.publish(msg)

        rospy.sleep(2)


    def StartRandomMission(self):

        self.ClearCostMaps()
        rospy.sleep(1)

        if not self.PreviusGoalCoords:
            self.PreviusGoalCoords = (-2.0, -0.5)  # Default starting point
        else:
            self.PreviusGoalCoords = self.GoalCoords

        self.GoalCoords = self.MissionPoints[random.randrange(len(self.MissionPoints))]

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.goal_coords[0]
        msg.pose.position.y = self.goal_coords[1]
        msg.pose.orientation.w = 1.0

        self.GoalPub.publish(msg)

        rospy.loginfo('[mission_planner_demo] Mission started towards: '+str(self.GoalCoords))


    def __mission_status_monitor(self, data):
        if data.status.status == 3:
            rospy.loginfo('[mission_planner_demo] Goal reached. Creating new mission.')
            self.StartRandomMission()

        elif data.status.status == 4:
            rospy.loginfo('[mission_planner_demo] Goal failed. ' + str(self.PreviusGoalCoords) +
                          ' -> ' +str(self.GoalCoords))
            
            # For simplicity, a new mission will be started.
            self.StartRandomMission()

if __name__ == '__main__':
    rospy.init_node('mission_planner_demo', anonymous=False)

    m = MissionPlanner()

    rospy.loginfo('[mission_planner] Started')

    m.StartRandomMission()

    try:
        while not rospy.is_shutdown():
            pass

    except Exception as e:
        print(e)

    rospy.loginfo("[mission_planner_demo] Stopped")
