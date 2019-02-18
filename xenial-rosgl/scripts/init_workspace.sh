#!/bin/bash

WORKDIR=/work
name_ros_version="kinetic"
name_catkin_workspace="catkin_ws"
name_svo_workspace="svo_install_ws"

echo "[Make the catkin workspace and test the catkin_make]"
echo "[Note] Catkin workspace   >>> $WORKDIR/$name_catkin_workspace"
cd $WORKDIR/$name_catkin_workspace
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$name_ros_version
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

#if [ -f $WORKDIR/$name_svo_workspace/install/setup.bash ]; then
#	catkin config --extend /work/svo_install_ws/install
#else
#	catkin config --extend /opt/ros/$name_ros_version
#fi


# if ~/.bashrc was set, exit here
if grep -q "ROS_PACKAGE_PATH" ~/.bashrc; then
	source ~/.bashrc
    exit 0
fi

echo "[Set the ROS evironment]"
sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
if [ -f $WORKDIR/$name_svo_workspace/install/setup.bash ]; then
	sh -c "echo \"source $WORKDIR/$name_svo_workspace/install/setup.bash\" >> ~/.bashrc"
fi
sh -c "echo \"source $WORKDIR/$name_catkin_workspace/devel/setup.bash\" >> ~/.bashrc"

sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"
sh -c "echo \"export ROS_PACKAGE_PATH=$WORKDIR:/opt/ros/$name_ros_version/share\" >> ~/.bashrc"

source ~/.bashrc

echo "[Complete!!!]"
exit 0

