FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
	&& echo '========== install basic apps ==========' \
	&& apt-get install -y build-essential gedit nano wget curl unzip cmake cmake-gui git mesa-utils \
	&& echo '========== install boost ==========' \
	&& apt-get install -y libboost-all-dev \
	&& echo '========== install eigen3, opencv ==========' \
	&& apt-get install -y libeigen3-dev libopencv-dev \
	&& echo '========== install pangolin dependencies ==========' \
	&& apt install -y libglew-dev libpython2.7-dev \
	&& apt install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
	&& apt install -y libdc1394-22-dev libraw1394-dev libuvc-dev libopenni2-dev \
	&& apt install -y libjpeg-dev libpng-dev libtiff-dev libopenexr-dev

