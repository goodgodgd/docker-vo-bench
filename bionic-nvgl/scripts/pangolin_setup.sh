#!/bin/bash

echo "========== apt upate =========="
apt update
apt upgrade -y

echo "========== apt install =========="
apt install -y libglew-dev libpython2.7-dev
apt install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
apt install -y libdc1394-22-dev libraw1394-dev libuvc-dev libopenni2-dev
apt install -y libjpeg-dev libpng-dev libtiff-dev libopenexr-dev
# apt install -y libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev	(ubuntu 16.04)

echo "========== build =========="
cd /work/project/Pangolin
rm -rf build || true
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
make install

cd /work/project
chmod -R a+rw ./

echo "[Complete!!!]"

