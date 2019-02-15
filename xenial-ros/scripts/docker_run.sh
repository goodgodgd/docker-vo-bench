#!/bin/bash

nvidia-docker run --name maplab -it --env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "/home/ian/workplace/docker-vo/xenial-ros/catkin_ws:/work/catkin_ws" \
	-v "/home/ian/workplace/docker-vo/xenial-ros/scripts:/work/scripts" \
	-v "/home/ian/workplace/docker-vo/dataset:/work/dataset" \
	-v "/home/ian/workplace/docker-vo/output:/work/output" \
	xenial-ros:dev bash

