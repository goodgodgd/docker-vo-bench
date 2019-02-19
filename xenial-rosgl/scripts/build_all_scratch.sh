#!/bin/bash

WORKDIR=/work
SCRIPT_PATH=$WORKDIR/scripts

echo -e "\n===== Remove all build files ======\n"
find $WORKDIR -name build -exec rm -rf {} +

echo -e "\n===== Build Pangolin ======\n"
$SCRIPT_PATH/pangolin_setup.sh

echo -e "\n===== Build ORB_SLAM2 ======\n"
$SCRIPT_PATH/orb2_setup.sh

echo -e "\n===== Build Ceres Solver ======\n"
$SCRIPT_PATH/ceres_setup.sh

echo -e "\n===== Build Maplab (ROVIOLI) ======\n"
$SCRIPT_PATH/init_workspace.sh
cd $WORKDIR/catkin_ws

# Ignore hand_eye_calibration in maplab_dependencies, 
# it makes build error but does not matter for maplab or ROVIOLI
touch $WORKDIR/catkin_ws/src/maplab_dependencies/internal/hand_eye_calibration/CATKIN_IGNORE
# Note: do NOT catkin_make since plain cmake projectes are included in maplab
catkin build maplab

echo -e "\n===== Build VinsFusion ======\n"
catkin build
catkin build vins

if [ -d "/work/svo_install_ws" ]; then
	echo -e "\n===== Build SVO2 ======\n"
	catkin build svo_ros
fi

