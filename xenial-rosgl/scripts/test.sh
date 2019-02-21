#!/bin/bash

/work/ORB_SLAM2/Examples/Stereo/stereo_euroc \
	/work/ORB_SLAM2/Vocabulary/ORBvoc.txt \
	/work/ORB_SLAM2/Examples/Stereo/TUM_VI.yaml \
    /data/dataset/tum_vi/dataset-room3_512_16/dso/cam0/images \
    /data/dataset/tum_vi/dataset-room3_512_16/dso/cam1/images \
    /work/ORB_SLAM2/Examples/Stereo/TumVI_TimeStamps/dataset-room3_512_16.txt \
    0 \
    /data/output/tum_vi.txt
