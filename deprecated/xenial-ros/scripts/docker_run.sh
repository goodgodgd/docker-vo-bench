#!/bin/bash

PROJECT=/home/ian/workplace/docker-vo

nvidia-docker run --name maplab -it --env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "$PROJECT/xenial-ros/catkin_ws:/work/catkin_ws" \
	-v "$PROJECT/xenial-ros/scripts:/work/scripts" \
	-v "$PROJECT/dataset:/work/dataset" \
	-v "$PROJECT/output:/work/output" \
	xenial-ros:dev bash

