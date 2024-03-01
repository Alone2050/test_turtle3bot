# test_turtle3bot

# Prerequisites
This is a ROS package that depends on the following:

Turtlebot3 package
Turtlebot3_simulation package
Rospy
Numpy
Please ensure the presence and installation of the above prior to installing this package. NOTE: This package was made and tested in ROS Noetic.

# Installation
Clone this repo to your catkin_ws/src

cd catkin_ws then catkin_make

source devel/setup.bash

chmod +x catkin_ws/src/test_turtle3bot/src/*.py 

# Usage

roslaunch test_turtle3bot SimpleMove.launch

dynamic obstacles -> roslaunch test_turtle3bot SimpleMove.launch add_obstacles:=true

RGBD camera roslaunch test_turtle3bot SimpleMove.launch add_obstacles:=true record_rgbd:=true
