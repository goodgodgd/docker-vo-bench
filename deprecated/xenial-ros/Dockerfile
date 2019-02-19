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
	&& echo -e '\n========== install maplab dependencies ==========' \
	&& apt install -y autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.8 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base

# git submodule update --init --recursive
# catkin build maplab

