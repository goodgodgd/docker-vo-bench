#!/usr/bin/env bash

DATASET="all"
NUM_TEST=2

source /work/maplab_ws/devel/setup.bash
python3 run_rovioli.py -d $DATASET -t $NUM_TEST
source /work/vins_ws/devel/setup.bash
python3 run_vinsfusion.py -d $DATASET -t $NUM_TEST
source /work/svo2_ws/devel/setup.bash
python3 run_svo2.py -d $DATASET -t $NUM_TEST

python3 run_orb2.py -e all -t $NUM_TEST
