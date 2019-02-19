#!/bin/bash

WORK_ROOT="/work"
ROS_VERSION="kinetic"
CATKIN_WS="catkin_ws"
SVO_WS="svo_install_ws"

echo "[Make the catkin workspace and test the catkin_make]"
echo "[Note] Catkin workspace   >>> $WORK_ROOT/$CATKIN_WS"
cd $WORK_ROOT/$CATKIN_WS

if [ -f $WORK_ROOT/$SVO_WS/install/setup.bash ]; then
	export CMAKE_PREFIX_PATH=$WORK_ROOT/$SVO_WS/install:$WORK_ROOT/$CATKIN_WS/devel:/opt/ros/$ROS_VERSION
else
	export CMAKE_PREFIX_PATH=$WORK_ROOT/$CATKIN_WS/devel:/opt/ros/$ROS_VERSION
fi

cd $WORK_ROOT/$CATKIN_WS
catkin clean
rm -rf .catkin_tools || true
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# if ~/.bashrc was set, exit here
if grep -q "ROS_PACKAGE_PATH" ~/.bashrc; then
	source ~/.bashrc
    exit 0
fi

echo "[Set the ROS evironment]"
sh -c "echo \"source /opt/ros/$ROS_VERSION/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source $WORK_ROOT/$CATKIN_WS/devel/setup.bash\" >> ~/.bashrc"
if [ -f $WORK_ROOT/$SVO_WS/install/setup.bash ]; then
	sh -c "echo \"source $WORK_ROOT/$SVO_WS/install/setup.bash\" >> ~/.bashrc"
	sh -c "echo \"export CMAKE_PREFIX_PATH=$WORK_ROOT/$SVO_WS/install:$WORK_ROOT/$CATKIN_WS/devel:/opt/ros/$ROS_VERSION\" >> ~/.bashrc"
else
	sh -c "echo \"export CMAKE_PREFIX_PATH=$WORK_ROOT/$CATKIN_WS/devel:/opt/ros/$ROS_VERSION\" >> ~/.bashrc"
fi

sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"
sh -c "echo \"export ROS_PACKAGE_PATH=$WORK_ROOT:/opt/ros/$ROS_VERSION/share\" >> ~/.bashrc"

source ~/.bashrc

echo "[Complete!!!]"
exit 0

