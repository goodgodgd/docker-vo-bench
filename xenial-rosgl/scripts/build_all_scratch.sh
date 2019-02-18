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
# catkin_make does not work since plain cmake projectes are included in maplab
catkin build maplab

echo -e "\n===== Build Vins, Svo ======\n"
# ignore hand_eye_calibration in maplab_dependencies, it makes build error but does not matter for maplab or ROVIOLI
touch $WORKDIR/catkin_ws/src/maplab_dependencies/internal/hand_eye_calibration/hand_eye_calibration_batch_estimation/CATKIN_IGNORE
# build all other packages: vins, svo_ros
catkin build

