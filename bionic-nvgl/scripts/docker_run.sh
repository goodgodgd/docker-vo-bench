#!/bin/bash

# xhost +local:docker

PROJECT=/home/ian/workplace/docker-vo

nvidia-docker run --name vo-bench -it --env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "$PROJECT/bionic-nvgl:/work/project" \
	-v "$PROJECT/dataset:/work/dataset" \
	-v "$PROJECT/output:/work/output" \
	bionic-nvgl:dev bash

# --rm: remove container after exit
# 시작 프로그램 추가: crontab -e 하고 아래줄 추가
# @reboot /bin/mount -t ntfs /dev/sdb1 /media/ian/iandata

