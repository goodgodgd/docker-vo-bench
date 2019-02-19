#!/bin/bash

echo "========== apt upate =========="
apt update

echo "========== apt install =========="
apt install -y unzip wget curl nano
# to solve error: libdc1394 error: Failed to initialize libdc1394
# https://stackoverflow.com/questions/42149458/open-cv-error-failed-to-init-raw1394-persisting-in-docker/42151836#42151836
apt install -y libdc1394-22-dev libdc1394-22 libdc1394-utils python-opencv
rm -rf /var/lib/apt/lists/*

echo "run echo ros package path:" $ROS_PACKAGE_PATH

# assuming that /work is shared with host and it contains ORB_SLAM
echo "========== download vocabulary =========="
cd /work/ORB_SLAM/Data
download_gdrive() {
	fileid=$1
	filename=$2
	curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=${fileid}" > /dev/null
	curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=${fileid}" -o ${filename}
	rm ./cookie
}

echo "========== build =========="
# build g2o
cd /work/ORB_SLAM/Thirdparty/g2o
mkdir build
cd build
echo "[current directory] " $PWD
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3

# build DBoW2
cd ../../DBoW2
mkdir build
cd build
echo "[current directory] " $PWD
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3

# build ORB_SLAM
cd /work/ORB_SLAM
# remove 'opencv' line in Indigo but still use opencv
sed -i '/opencv/d' manifest.xml
mkdir build
cd build
echo "[current directory] " $PWD
cmake .. -DROS_BUILD_TYPE=Release
make -j3

