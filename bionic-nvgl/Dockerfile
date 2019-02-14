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


# xhost +local:docker
# nvidia-docker run --name vo-bench -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/home/ian/workspace/docker/bionic-nvgl:/work" -v "/media/ian/iandata/datasets:/work/data" bionic-nvgl:dev bash
# nvidia-docker run --name vo-bench2 -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/home/ian/workplace/docker-vo/bionic-nvgl:/work" bionic-nvgl:dev2 bash

# --rm: remove container after exit

# crontab -e 하고 아래줄 추가
# @reboot /bin/mount -t ntfs /dev/sdb1 /media/ian/iandata
# @reboot /usr/bin/xhost +local:docker


