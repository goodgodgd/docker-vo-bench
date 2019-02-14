#!/bin/bash

echo "========== apt upate =========="
apt update
#apt upgrade -y

echo "========== apt install =========="
apt install -y libeigen3-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev
# rm -rf /var/lib/apt/lists/*

echo "========== build =========="
export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/Pangolin
ORB2_ROOT=/work/ORB_SLAM2
git checkout ian-vo-bench

echo -e "\nConfiguring and building Thirdparty/g2o ..."

cd $ORB2_ROOT
cd Thirdparty/g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo -e "\nConfiguring and building Thirdparty/DBoW2 ..."

cd $ORB2_ROOT
cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo -e "\nConfiguring and building ORB_SLAM2 ..."

cd $ORB2_ROOT
# rm -rf build || true
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# assuming that /work is shared with host and it contains ORB_SLAM
echo -e "\n========== download vocabulary =========="
cd $ORB2_ROOT
mkdir -p Vocabulary
cd ${ORB2_ROOT}/Vocabulary

download_gdrive() {
	fileid=$1
	filename=$2
	curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=${fileid}" > /dev/null
	curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=${fileid}" -o ${filename}
	rm ./cookie
}

if [ ! -f ORBvoc.txt ]; then
	download_gdrive "1TeCFP7ykKMW6mYLZPT_tfvb3eY14VuBR" "ORBvoc.txt"
fi

cd /work
chmod -R a+rw ./

echo "[Complete!!!]"

