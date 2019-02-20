# vo-bench


download tum vi dataset
```
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off https://cdn3.vision.in.tum.de/tumvi/calibrated/512_16/
wget -R "index.*" -m -np -nH --no-check-certificate -e robots=off https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/
```

```
# added submodules
git submodule add https://github.com/stevenlovegrove/Pangolin.git xenial-rosgl/Pangolin
git submodule add https://github.com/goodgodgd/ORB_SLAM2.git xenial-rosgl/ORB_SLAM2
git submodule add https://ceres-solver.googlesource.com/ceres-solver xenial-rosgl/ceres_solver
git submodule add https://github.com/ethz-asl/maplab_dependencies.git xenial-rosgl/maplab_ws/src/maplab_dependencies
git submodule add https://github.com/goodgodgd/maplab.git xenial-rosgl/maplab_ws/src/maplab
git submodule add https://github.com/goodgodgd/VINS-Fusion.git xenial-rosgl/vins_ws/src/vins-fusion
git submodule add https://github.com/goodgodgd/rpg_svo_example.git xenial-rosgl/svo2_ws/src/rpg_svo_example
git submodule add https://github.com/catkin/catkin_simple.git xenial-rosgl/svo2_ws/src/catkin_simple

# pull all submodules recursively
git submodule update --init --recursive
```

