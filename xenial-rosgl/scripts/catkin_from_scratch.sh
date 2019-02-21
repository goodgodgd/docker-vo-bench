#!/bin/bash

WORK_ROOT="/work"
ROS_VERSION="kinetic"
MAPLAB_WS="$WORK_ROOT/maplab_ws"
VINS_WS="$WORK_ROOT/vins_ws"
SVO2_WS="$WORK_ROOT/svo2_ws"

if [ -d "$SVO2_WS" ]; then
	echo "[Fix paths for SVO2]"
	cd $SVO2_WS
	mv install devel
	grep -IRl "/home/zichao/svo_install_ws" devel/ | xargs sed -i "s:/home/zichao/svo_install_ws:/work/svo2_ws:g"
fi
ag "/home/zichao/svo_install_ws"
sleep 2

catkin_init_ws_and_build() {
	echo -e "\n[Make $1 workspace] >>> $2"
	cd $2
	ls
	if [ $3 -eq 1 ]; then
		rm -rf devel || true
		echo "remove devel $3"
		sleep 3
	fi
	rm -rf install logs .catkin_tools || true
	catkin init
	catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
	catkin config --extend /opt/ros/kinetic
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

	echo -e "\n[Build $1]"
	catkin build $4
	catkin build $4
	echo -e "[Build finished]"
	sleep 5
}

catkin_init_ws_and_build "svo2" $SVO2_WS 0 ""
catkin_init_ws_and_build "maplab" $MAPLAB_WS 1 "maplab"
catkin_init_ws_and_build "vins" $VINS_WS 1 ""

echo "[Set the ROS evironment]"

sh -c "echo \"export LC_ALL=C.UTF-8\" >> ~/.bashrc"
sh -c "echo \"export LANG=C.UTF-8\" >> ~/.bashrc"
sh -c "echo \"source /opt/ros/$ROS_VERSION/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source $MAPLAB_WS/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source $VINS_WS/devel/setup.bash\" >> ~/.bashrc"
if [ -f $SVO2_WS/install/setup.bash ]; then
	sh -c "echo \"source $SVO2_WS/install/setup.bash\" >> ~/.bashrc"
fi

sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"
sh -c "echo \"export ROS_PACKAGE_PATH=$WORK_ROOT:/opt/ros/$ROS_VERSION/share\" >> ~/.bashrc"

source ~/.bashrc
echo "[Complete!!!]"
exit 0



echo -e "\n[Make maplab workspace] >>> $MAPLAB_WS"
cd $MAPLAB_WS
rm -rf install devel logs .catkin_tools || true
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

echo -e "\n[Build Maplab]"
catkin build maplab
if [ ! -d "$MAPLAB_WS/devel/lib/rovioli" ]; then
	echo "Maplab setup failed"
	exit 1
fi


echo -e "\n[Make VinsFusion workspace] >>> $VINS_WS"
cd $VINS_WS
rm -rf install devel logs .catkin_tools || true
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

echo -e "\n[Build Vins]"
catkin build
if [ ! -d "$VINS_WS/devel/lib/vins" ]; then
	echo "Vins setup failed"
	exit 1
fi


if [ ! -d "$SVO2_WS" ]; then
	exit 0
fi

echo -e "\n[Make SVO2 workspace] >>> $VINS_WS"
cd $SVO2_WS
rm -rf install devel logs .catkin_tools || true
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

echo -e "\n[Build SVO2]"
catkin build
if [ ! -d "$SVO2_WS/devel/lib/svo" ]; then
	echo "SVO2 setup failed"
	exit 1
fi
exit 0

