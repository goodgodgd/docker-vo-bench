!#/bin/bash

cd /work/ceres_solver
mkdir ceres-bin
cd ceres-bin
cmake ..
make -j4
# make test
make install

