FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04

ENV DEBIAN_FRONTEND=noninteractive
COPY scripts/ros_kinetic_setup.sh /root
RUN apt-get update \
	&& apt-get upgrade -y \
	&& echo -e '\n========== install ros ==========' \
	&& chmod a+x /root/ros_kinetic_setup.sh \
	&& /root/ros_kinetic_setup.sh \
	&& echo -e '\n========== install basic apps ==========' \
	&& apt-get install -y build-essential gedit nano wget curl unzip cmake cmake-gui git mesa-utils \
	&& echo '========== install boost ==========' \
	&& apt-get install -y libboost-all-dev \
	&& echo '========== install eigen3, opencv ==========' \
	&& apt-get install -y libeigen3-dev libopencv-dev \
	&& echo '========== install pangolin dependencies ==========' \
	&& apt install -y libglew-dev libpython2.7-dev libpython3.5-dev \
	&& apt install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
	&& apt install -y libdc1394-22-dev libraw1394-dev libopenni2-dev \
	&& apt install -y libjpeg-dev libpng-dev libtiff-dev libopenexr-dev \
	&& echo -e '\n========== install maplab dependencies ==========' \
	&& apt install -y autotools-dev ccache doxygen dh-autoreconf git 
	&& apt install -y liblapack-dev libblas-dev libgtest-dev libv4l-dev \
	&& apt install -y libreadline-dev libssh2-1-dev pylint clang-format-3.8 python-autopep8 python-catkin-tools \
	&& apt install -y python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base \
	&& echo '========== install ceres dependencies ==========' \
	&& apt install -y libeigen3-dev libsuitesparse-dev libgoogle-glog-dev libatlas-base-dev libtbb-dev


# pangolin dependency (optional): libuvc (For cross-platform webcam video input via libusb)
# 	install from source "https://github.com/ktossell/libuvc.git" in xeinal or "apt install libuvc-dev" in bionic

