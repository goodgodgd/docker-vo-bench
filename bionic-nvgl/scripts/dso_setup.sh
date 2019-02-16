#!/bin/bash

echo "========== apt upate =========="
apt update

echo "========== apt install =========="
apt install -y libsuitesparse-dev libeigen3-dev libboost-all-dev python3-pip
pip3 install ruamel.yaml

echo "========== install ziplib =========="

if [ ! -f /usr/local/include/zipconf.h ]; then
	apt install -y zlib1g-dev
	cd /work/project/dso/thirdparty
	tar -zxvf libzip-1.1.1.tar.gz
	cd libzip-1.1.1/
	./configure
	make -j4
	make install
	sudo cp lib/zipconf.h /usr/local/include/zipconf.h
fi

echo "========== build dso =========="
cd /work/project/dso
rm -rf build || true
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd /work/project
chmod -R a+rw ./

echo "[Complete!!!]"

