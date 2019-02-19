#!/bin/bash

PROJECT=/home/ian/workspace/docker-vo
DATASET=/media/ian/iandata/datasets

nvidia-docker run --name vo-bench -it --env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "$PROJECT/xenial-rosgl:/work" \
	-v "$DATASET:/data/dataset" \
	-v "$PROJECT/output:/data/output" \
	xenial-rosgl:dev bash

