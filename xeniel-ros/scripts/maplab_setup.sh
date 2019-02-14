#!/bin/bash

apt update -y

echo -e "\n===== Install framework dependencies. ====="
# NOTE: clang-format-3.8 is not available anymore on bionic, install a newer version.
apt install -y autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.9 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base

pip install requests


echo -e "\n===== Create catkin workspace. ====="
export ROS_VERSION=melodic
export CATKIN_WS=/work/catkin/maplab_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
