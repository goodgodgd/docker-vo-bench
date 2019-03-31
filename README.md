# Scale-Aware Visual Odometry Benchmark

This repository holds a benchmark system for scale aware visual odometry in a docker image. There are several sensor configurations for visual odometry as following
1. Monocular Visual Odometry: [ORB-SLAM](https://github.com/raulmur/ORB_SLAM) (turning off loop closure) and [DSO](https://github.com/JakobEngel/dso) belong to here. Since only monocular camera is used, they are scale-ignorant. We don't handle this configuration.
2. Stereo Visual Odometry: [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) supports this configuration.
3. Monocular Visual-Inertial Odometry (MVIO): [ROVIOLI](https://github.com/ethz-asl/maplab) in maplab belongs to here. 
4. Stereo Visual-Inertial Odometry (MVIO): [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) supports this configuration.
5. Types 2~4 are scale-aware and thus our evaluation target. The latest visual odometry algorithms except for type 1 are evaluated in this benchmark system. The selected algorithms have publicly available source codes. Table 1 shows algorithm abbreviations and supported sensor configurations evaluated in this benchmark.

| Algorithm                                                    | Stereo | MVIO | SVIO |
| ------------------------------------------------------------ | ------ | ---- | ---- |
| [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)            | O      |      |      |
| [ROVIOLI](https://github.com/ethz-asl/maplab)                |        | O    |      |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | O      | O    | O    |
| [SVO2](http://rpg.ifi.uzh.ch/svo2.html)                      | O      | O    | O    |


Thus total 8 algorithms are evaluated here now: ORB-SLAM2 (Stereo), ROVIOLI (MVIO), VINS-Fusion (Stereo, MVIO, SVIO), SVO2 (Stereo, MVIO, SVIO)  

We provide a docker image to install dependencies in an isolated environment independent of the host OS. There are **5 steps** to run our benchmark system.

> Note: `.` will always mean the repository root dir.

## 1. Build Docker Image

If you have not installed docker, you have to install docker. ([Ubuntu install](https://docs.docker.com/v17.09/engine/installation/linux/docker-ce/ubuntu/), [Windows Install](https://docs.docker.com/v17.09/docker-for-windows/install/)). Here are shortcuts for ubunt installation.

```bash
# remove old versions
$ sudo apt-get remove docker docker-engine docker.io

# install required pakages
$ sudo apt-get update
$ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common

# add docker's official GPG key
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
# verify the key, see the output looks like below
$ sudo apt-key fingerprint 0EBFCD88
pub   4096R/0EBFCD88 2017-02-22
      Key fingerprint = 9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88
uid                  Docker Release (CE deb) <docker@docker.com>
sub   4096R/F273FCD8 2017-02-22

# add repository
$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
$ sudo apt-get update
$ sudo apt-get install docker-ce

# to use docker without sudo
sudo usermod -aG docker $USER
```

To build docker, you need a `Dockerfile`. We made one for our benchmark system. Let's take a look at `./xenial-rosgl/Dockerfile`

```bash
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
...
```

It is inherited from `nvidia/opengl:1.0-glvnd-devel-ubuntu16.04`. The parent image enables OpenGL GUI applications in a docker container. Its base OS is Ubuntu 16.04. The `RUN` instruction installs all the dependencies of the selected algorithms. The main dependency is `ROS (Kinetic Kame)`.  Since installation of ROS requires a number of lines of commands, it was replaced by the script `./xenial-rosgl/scripts/ros_kinetic_setup.sh`. 

Then let's build the image by

```bash
# you can change [image name]:[tag]
cd xenial-rosgl
docker build . -t xenial-rosgl:dev
```

## 2. Create Docker Container

The basic command to create a docker container from an image is like

```bash
docker run -it --name [container name] [image name]:[tag] bash
```

However, to enable GUI applications and share algorithms source codes in this repository with  the container, you need to give more options like `./xenial-rosgl/scripts/docker_run.sh`

```bash
#!/bin/bash
# NOTE: you need to replace paths here
PROJECT=/home/ian/workspace/docker-vo
DATASET=/media/ian/iandata/datasets

nvidia-docker run --name savo-bench -it --env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "$PROJECT/xenial-rosgl:/work" \
	-v "$DATASET:/data/dataset" \
	-v "$PROJECT/output:/data/output" \
	xenial-rosgl:dev bash
```

`-v` or `--volume` option is to share folders with the host OS. You have to understand this command and **replace the `PROJECT` and `DATASET` paths with your host paths**. Leave `--env="DISPLAY"` and `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"` as they enable GUI. If you run the script, the terminal is connected to the container. If you want to leave container, run `exit`.

One more thing you have to give attention is **"always  run `xhost +local:docker` before start the container"**. Without `xhost +local:docker`, GUI applications cannot start in the docker container.

When you create a container, just run
```bash
xhost +local:docker
./xenial-rosgl/scripts/docker_run.sh
```

When you restart the container, run

```bash
xhost +local:docker
# check container's status 
docker ps -a
# if container status is "Exited"
docker start vo-bench
# enter into container's terminal
docker exec -it vo-bench bash
```

written upto here

---

## 3. Build Source Codes

Open-source algorithms were forked and added to this project as submodules. As they have already been added, **you do NOT have to run the following commands.**

```bash
git submodule add https://github.com/stevenlovegrove/Pangolin.git xenial-rosgl/Pangolin
git submodule add https://github.com/goodgodgd/ORB_SLAM2.git xenial-rosgl/ORB_SLAM2
git submodule add https://ceres-solver.googlesource.com/ceres-solver xenial-rosgl/ceres_solver
git submodule add https://github.com/ethz-asl/maplab_dependencies.git xenial-rosgl/maplab_ws/src/maplab_dependencies
git submodule add https://github.com/goodgodgd/maplab.git xenial-rosgl/maplab_ws/src/maplab
git submodule add https://github.com/goodgodgd/VINS-Fusion.git xenial-rosgl/vins_ws/src/vins-fusion
git submodule add https://github.com/goodgodgd/rpg_svo_example.git xenial-rosgl/svo2_ws/src/rpg_svo_example
git submodule add https://github.com/catkin/catkin_simple.git xenial-rosgl/svo2_ws/src/catkin_simple
```

Instead, you have to pull all submodules recursively.

```bash
git submodule update --init --recursive
```

The submodules are built by the following scripts **inside the docker container.**

```bash
cd /work/scripts
# build pangolin, ORB SLAM2, Ceres solver
./build_from_scratch.sh
# build SVO2, MAPLAB, VINS-fusion
./catkin_from_scratch.sh
```

## 4. Prepare Datasets

EuRoC MAV dataset can be downloaded [here](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). As there are just 11 sequences, you can click one by one to download. Since ORB SLAM2 consumes image files and the others consume rosbag files, the both formats have to be downloaded. You have to move the downloaded files to `$DATASET/euroc_mav` and extract `.tar` files where `DATASET` was defined in `./xenial-rosgl/scripts/docker_run.sh`.  

You may download TUM VI dataset from the [homepage](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) but there are 28 sequences, it will take too much time. Instead, you'd be better off using the following scripts. Then you have to move the downloaded files to `$DATASET/tum_vi` and extract `.tar` files   where `DATASET` was defined in `./xenial-rosgl/scripts/docker_run.sh`.  

```bash
# download rosbag files
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off https://cdn3.vision.in.tum.de/tumvi/calibrated/512_16/
# download image files
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/
```

## 5. Run Algorithms by Scripts

For evaluation each algorithm is executed for every dataset sequence five times each. These iterative executions are generated by the python scripts in `./xenial-rosgl/scripts`.  You can run every algorithm for every sequence by running a single script `/work/scripts/run_all.sh`  **inside the container.** The script was written as follows. If you failed in the middle of the process, you may start from the failed python file.

```bash
source /work/#!/usr/bin/env bash

DATASET="all"
NUM_TEST=5

source /work/maplab_ws/devel/setup.bash
python3 run_rovioli.py -d $DATASET -t $NUM_TEST
source /work/vins_ws/devel/setup.bash
python3 run_vinsfusion.py -d $DATASET -t $NUM_TEST
source /work/svo2_ws/devel/setup.bash
python3 run_svo2.py -d $DATASET -t $NUM_TEST

python3 run_orb2.py -e all -t $NUM_TEST
```

## 6. Evaluate Results

Once you executed `run_all.sh`, there would be hundreds of pose files in `$PROJECT/output/pose` where `PROJECT` was defined in `docker_run.sh`. Evaluation scripts are placed in `./tools`. There are five steps to evaluate the results.

### 6.1. Define paths

Because I hate to type long paths in command lines, important paths are defined in the python script, `./tools/define_paths.py`. However, the file is being ignored by git and hence not in the repository. Instead, there is an example file, `./tools/define_paths-example.py` . You have to copy the example file as `./tools/define_paths.py` and edit the path variables in it to be adapted in your PC. 

### 6.2 Create GT Poses

To evaluate estimated poses, the ground truth (GT) poses are required. The GT pose files are scattered over the dataset structure. `create_gt_poses.py` finds all the pose files and save them as `csv` files in `./output/ground_truth`. Simply run the following command.

```bash
python3 ./tools/evaluation/create_gt_poses.py
```

### 6.3 Convert Estimated Poses

The ground truth poses are usually body-frame poses rather than camera-frame but the algorithms estimated the camera poses. As we know the extrinsics between camera and body, we have to transform the estimated poses into the body frame. The following command transforms poses and save the results into `./output/pose_body`.

```bash
python3 ./tools/evaluation/convert_to_body_pose.py
```

### 6.4 Evaluate and Draw Graphs

Once the GT poses and the estimated body poses are prepared, you can compute the evaluation metrics: Absolute Translational Error (ATE), Relative Pose Error (RPE), Frame Processing Time (FPT). The evaluation and drawing graphs are done by the following commands.

```bash
python3 ./tools/evaluation/evaluate_ate_all
python3 ./tools/evaluation/evaluate_rpe_all
python3 ./tools/evaluation/evaluate_time
python3 ./plotters/plot_errors.py
python3 ./plotters/plot_timings.py
```

