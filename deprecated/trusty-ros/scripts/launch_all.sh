#!/bin/bash

# launch roscore / rosrun / rosbag scripts in docker in a new terminal tab each

gnome-terminal --tab -- sh -c "docker exec -it orbslam4 bash -c 'source /work/roscore.sh'"
sleep 3
gnome-terminal --tab -- sh -c "docker exec -it orbslam4 bash -c 'source /work/orbslam.sh'"
sleep 12
gnome-terminal --tab -- sh -c "docker exec -it orbslam4 bash -c 'source /work/rosbag.sh'"

