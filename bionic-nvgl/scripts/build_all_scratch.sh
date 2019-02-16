#!/bin/bash

PROJECT=/work/project
find $PROJECT -name build -exec rm -rf {} +
echo -e "\n===== Start buidling Pangolin ======\n"
$PROJECT/scripts/pangolin_setup.sh
echo -e "\n===== Start buidling DSO ======\n"
$PROJECT/scripts/dso_setup.sh
echo -e "\n===== Start buidling ORB_SLAM2 ======\n"
$PROJECT/scripts/orb2_setup.sh

