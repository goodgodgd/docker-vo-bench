#!/bin/bash

WORK_ROOT=/work
SCRIPT_PATH=$WORK_ROOT/scripts

echo -e "\n===== Remove all build files ======\n"
find $WORK_ROOT -name build -exec rm -rf {} +

echo -e "\n===== Build Pangolin ======\n"
$SCRIPT_PATH/pangolin_setup.sh
if [ ! -f "/usr/local/lib/libpangolin.so" ]; then
	echo "Pangolin setup failed"
	exit 1
fi

echo -e "\n===== Build ORB_SLAM2 ======\n"
$SCRIPT_PATH/orb2_setup.sh
if [ ! -f "/work/ORB_SLAM2/lib/libORB_SLAM2.so" ]; then
	echo "Pangolin setup failed"
	exit 1
fi

echo -e "\n===== Build Ceres Solver ======\n"
$SCRIPT_PATH/ceres_setup.sh
if [ ! -f "/usr/local/lib/libceres.a" ]; then
	echo "Ceres solver setup failed"
	exit 1
fi

exit 0




echo -e "\n===== Init catkin workspace ======\n"
$SCRIPT_PATH/init_workspace.sh
if grep -q "ROS_PACKAGE_PATH" ~/.bashrc; then
	echo ""
else
	echo "Init catkin workspace failed"
    exit 1
fi

echo -e "\n===== Build Maplab (ROVIOLI) ======\n"
cd $CATKIN_WS
# Note: do NOT catkin_make since plain cmake projectes are included in maplab
# Note: do NOT "catkin build" since some packages (out of maplab scope) in maplab_dependencies yields build error
catkin build maplab
if [ ! -f "/work/catkin_ws/devel/lib/rovioli/rovioli" ]; then
	echo "Maplab (ROVIOLI) setup failed"
	exit 1
fi

echo -e "\n===== Build VinsFusion ======\n"
catkin build vins

if [ -d "/work/svo_install_ws" ]; then
	echo -e "\n===== Build SVO2 ======\n"
	catkin build svo_ros
fi

