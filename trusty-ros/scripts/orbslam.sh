#!/bin/bash

source /opt/ros/indigo/setup.bash
source /work/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_PACKAGE_PATH=/work:/opt/ros/indigo/share:/opt/ros/indigo/stacks
rosrun ORB_SLAM ORB_SLAM Data/ORBVoc.txt Data/Settings.yaml 0 Example

