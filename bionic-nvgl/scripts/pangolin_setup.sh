#!/bin/bash

echo "========== apt upate =========="
apt update
apt upgrade -y

echo "========== apt install =========="
apt install -y libglew-dev libpython2.7-dev
apt install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
apt install -y libdc1394-22-dev libraw1394-dev libuvc-dev libopenni2-dev
apt install -y libjpeg-dev libpng-dev libtiff-dev libopenexr-dev
# pangolin dependency (optional): libuvc (For cross-platform webcam video input via libusb)
# 	install from source "https://github.com/ktossell/libuvc.git" in xeinal or "apt install libuvc-dev" in bionic

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

