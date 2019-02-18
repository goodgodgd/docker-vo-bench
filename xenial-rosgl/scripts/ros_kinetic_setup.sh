#!/bin/bash
# ref: http://wiki.ros.org/kinetic/Installation/Ubuntu

echo ""
echo "[Note] Target OS version  >>> Ubuntu 16.04.x (bionic)"
echo "[Note] Target ROS version >>> ROS Kinetic"
echo ""

echo "[Set the target OS, ROS version and name of catkin workspace]"
WORKDIR=/work
name_os_version=${name_os_version:="xenial"}
name_ros_version=${name_ros_version:="kinetic"}

echo "[Update the package lists and upgrade them]"
apt update -y
apt upgrade -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
apt install -y chrony ntpdate build-essential
ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
  cat /etc/apt/sources.list.d/ros-latest.list
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "ROS Builder"`
if [ -z "$roskey" ]; then
  apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "ROS Builder"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo -e "\n[Update the package lists and upgrade them]"
apt update
apt upgrade -y

echo -e "\n[Install the ros-desktop-full and all rqt plugins]"
apt install -y ros-$name_ros_version-desktop-full 
echo -e "\n[Install all rqt plugins]"
apt install -y ros-$name_ros_version-rqt-*

echo -e "\n[Initialize rosdep]"
rosdep init
rosdep update

echo -e "\n[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools

echo "[Complete!!!]"
exit 0

