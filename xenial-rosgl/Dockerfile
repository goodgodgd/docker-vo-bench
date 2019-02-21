FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04

ENV DEBIAN_FRONTEND=noninteractive
COPY scripts/ros_kinetic_setup.sh /root
RUN apt-get update \
	&& apt-get upgrade -y \
	&& echo -e '\n========== install ros ==========' \
	&& chmod a+x /root/ros_kinetic_setup.sh \
	&& /root/ros_kinetic_setup.sh \
	&& echo -e '\n========== install basic apps ==========' \
	&& apt-get install -y build-essential gedit nano wget curl unzip cmake git mesa-utils \
	&& echo -e '\n========== install pythons ==========' \
	&& apt-get install -y libpython2.7-dev libpython3.5-dev python-pip python3-pip python3-pandas python3-numpy \
	&& echo '========== install boost ==========' \
	&& apt-get install -y libboost-all-dev \
	&& echo '========== install eigen3, opencv ==========' \
	&& apt-get install -y libeigen3-dev libopencv-dev \
	&& echo '========== install pangolin dependencies ==========' \
	&& apt install -y libglew-dev  \
	&& apt install -y ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
	&& apt install -y libdc1394-22-dev libraw1394-dev libopenni2-dev \
	&& apt install -y libjpeg-dev libpng-dev libtiff-dev libopenexr-dev \
	&& echo -e '\n========== install maplab dependencies ==========' \
	&& apt install -y autotools-dev ccache doxygen dh-autoreconf clang-format-3.8 \
	&& apt install -y liblapack-dev libblas-dev libgtest-dev libv4l-dev libatlas3-base \
	&& apt install -y libreadline-dev libssh2-1-dev pylint python-autopep8 python-catkin-tools \
	&& apt install -y python-git python-setuptools python-termcolor python-wstool \
	&& echo '========== install ceres dependencies ==========' \
	&& apt install -y libeigen3-dev libsuitesparse-dev libgoogle-glog-dev libatlas-base-dev libtbb-dev \
	&& echo -e '\n========== install optional utils ==========' \
	&& apt-get install -y rpl silversearcher-ag cmake-gui \
	&& pip3 install pipenv

