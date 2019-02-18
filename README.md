# vo-bench


```
catkin build maplab
```

download tum vi dataset
```
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off https://cdn3.vision.in.tum.de/tumvi/calibrated/512_16/
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/
```

```
# add submodules
git submodule add https://github.com/stevenlovegrove/Pangolin.git xenial-rosgl/Pangolin
git submodule add https://github.com/goodgodgd/ORB_SLAM2.git xenial-rosgl/ORB_SLAM2
git submodule add https://ceres-solver.googlesource.com/ceres-solver xenial-rosgl/ceres_solver
git submodule add https://github.com/ethz-asl/maplab_dependencies.git xenial-rosgl/catkin_ws/src/maplab_dependencies
git submodule add https://github.com/goodgodgd/maplab.git xenial-rosgl/catkin_ws/src/maplab
git submodule add https://github.com/goodgodgd/VINS-Fusion.git xenial-rosgl/catkin_ws/src/vins-fusion
git submodule add https://github.com/goodgodgd/rpg_svo_example.git xenial-rosgl/catkin_ws/src/rpg_svo_example

# pull all submodules recursively
git submodule update --init --recursive
```

setup scripts
```
cd /work/scripts
./pangolin_setup.sh
./orb2_setup.sh
./ceres_setup.sh
./init_workspace.sh
cd /work/catkin_ws
# catkin_make does not work since plain cmake projectes are included in maplab
catkin build maplab
touch /work/catkin_ws/src/maplab_dependencies/internal/hand_eye_calibration/hand_eye_calibration_batch_estimation/CATKIN_IGNORE
catkin build
```

