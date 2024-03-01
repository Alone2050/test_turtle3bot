#!/usr/bin/python

import rospy
import time
import threading
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

class ObstacleController:

    def __init__(self, id, start_point, end_point, dst_step=200, time_step=0.2):

        self.id = id
        self.dstStep = dst_step
        self.TimeStep = time_step
        self.Thread = 0

        self.State = 1  # 0: stopped, 1: active

        # Create motion trajectory
        tx = np.pad([start_point[0]], (0, dst_step), mode='linear_ramp', end_values=(end_point[0]))
        ty = np.pad([start_point[1]], (0, dst_step), mode='linear_ramp', end_values=(end_point[1]))
        self.trajectory = np.vstack((tx,ty)).T

        # Initialize the obstacle object in Gazebo
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        SpawnModel = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        SpawnModel(
            model_name='unit_cylinder_'+str(self.id),
            model_xml=open('../sdf/unit_cylinder/model.sdf', 'r').read(),
            initial_pose=Pose(),
            reference_frame='map'
        )

        self.PoseSetPub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        while self.PoseSetPub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect")
            rospy.sleep(1)

        msg = ModelState()
        msg.model_name = 'unit_cylinder_'+str(self.id)
        msg.pose.position.x = start_point[0]
        msg.pose.position.y = start_point[1]
        self.pose_set_pub.publish(msg)


    def __motion(self):

        while self.state == 1:

            msg = ModelState()

            for i in np.arange(self.trajectory.shape[0]):
                msg.model_name = 'unit_cylinder_' + str(self.id)
                msg.pose.position.x = self.trajectory[i][0]
                msg.pose.position.y = self.trajectory[i][1]
                self.pose_set_pub.publish(msg)
                time.sleep(self.time_step)

            for i in np.arange(self.trajectory.shape[0]-1, -1, -1):
                msg.model_name = 'unit_cylinder_' + str(self.id)
                msg.pose.position.x = self.trajectory[i][0]
                msg.pose.position.y = self.trajectory[i][1]
                self.pose_set_pub.publish(msg)
                time.sleep(self.time_step)

        return

    def StartMotion(self):
        self.Thread = threading.Thread(target=self.__motion)
        self.Thread.start()

    def StopMotion(self):
        self.Ttate = 0
        self.Thread.join()

if __name__ == '__main__':

    rospy.init_node('obstacle_controller_demo', anonymous=False)

    o1 = ObstacleController(1, [1.0, 1.75], [-1.0, 1.75])
    o2 = ObstacleController(2, [0.5, 0.5], [0.5, -1.75])

    o1.StartMotion()
    o2.StartMotion()

    try:
        while not rospy.is_shutdown():
            pass

    except Exception as e:
        o1.StopMotion()
        o2.StopMotion()
        print(e)

    rospy.loginfo("[obstacle_controller_demo] Stopped")
