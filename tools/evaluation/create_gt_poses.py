import os
import os.path as op
import pandas as pd
import glob
import yaml
import numpy as np
import evaluation.rotation as rotation

import settings
from define_paths import *
import sequence_abbrev as sa

# Note: Set EUROC_PATH, OUTPUT_PATH in defins_paths.py before run this


def convert_euroc(srcpath, dstpath):
    seq_paths = [s.rstrip("/") for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    result_path = op.join(dstpath, "ground_truth", "euroc_mav")
    os.makedirs(result_path, exist_ok=True)
    for seq_path in seq_paths:
        print("sequence", seq_path)
        data = pd.read_csv(op.join(seq_path, "mav0/state_groundtruth_estimate0/data.csv"))
        camf = open(op.join(seq_path, "mav0/cam0/sensor.yaml"))
        cam_param = yaml.load(camf)
        cam2body = np.array(cam_param['T_BS']['data']).reshape((4, 4))
        print("camera pose to body\n", cam2body)

        # extract required columns: timestamp, x, y, z, qx, qy, qz, qw
        req_columns = ['#timestamp', ' p_RS_R_x [m]', ' p_RS_R_y [m]', ' p_RS_R_z [m]',
                       ' q_RS_x []', ' q_RS_y []', ' q_RS_z []', ' q_RS_w []']
        data = data[req_columns]

        # rename columns
        new_names = ["#timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
        renamer = dict(zip(req_columns, new_names))
        data = data.rename(columns=renamer)

        seq_abbr = sa.euroc_abbrev(op.basename(seq_path))
        savefile = op.join(result_path, seq_abbr + ".csv")
        process_and_save(data, cam2body, savefile, "%1.06f")


def convert_tumvi(srcpath, dstpath):
    seq_paths = [s.rstrip("/") for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    result_path = op.join(dstpath, "ground_truth", "tum_vi")
    os.makedirs(result_path, exist_ok=True)
    for seq_path in seq_paths:
        data = pd.read_csv(op.join(seq_path, "mav0/mocap0/data.csv"))
        camf = open(op.join(seq_path, "dso/camchain.yaml"))
        cam_param = yaml.load(camf)
        cam2body = np.array(cam_param['cam0']['T_cam_imu']).reshape((4, 4))
        print("camera pose to body\n", cam2body)

        # rename columns
        new_names = ["#timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
        renamer = dict(zip(list(data), new_names))
        data = data.rename(columns=renamer)

        seq_abbr = sa.tumvi_abbrev(op.basename(seq_path))
        savefile = op.join(result_path, seq_abbr + ".csv")
        process_and_save(data, cam2body, savefile, "%1.09f")


def transform_to_campose(data, cam2body_mat):
    print("before\n", data.iloc[:2, 1:])
    for i, ind in enumerate(data.index):
        timestamp = data.loc[ind, "#timestamp"]
        pose_body = data.loc[ind, :].values
        body_mat = rotation.transform44(pose_body)
        new_poseq = rotation.pose_quat(np.dot(body_mat, cam2body_mat), timestamp)
        data.loc[ind, :] = new_poseq
    print("after\n", data.iloc[:2, 1:])
    return data


def process_and_save(data, cam2body, savefile, float_format):
    # gt poses are body poses, transform them to be camera poses
    data = transform_to_campose(data, cam2body)

    # nanosecond timestamp to second
    data["#timestamp"] = data["#timestamp"] / 1.0E+9
    data = data.set_index("#timestamp")

    # save file
    print("save to", savefile)
    print(data.head())
    data.to_csv(savefile, encoding="utf-8", float_format=float_format)


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    assert op.isdir(EUROC_PATH), "No dir: " + EUROC_PATH
    assert op.isdir(TUMVI_PATH), "No dir: " + TUMVI_PATH
    assert op.isdir(OUTPUT_PATH), "No dir: " + OUTPUT_PATH

    convert_euroc(EUROC_PATH, OUTPUT_PATH)
    convert_tumvi(TUMVI_PATH, OUTPUT_PATH)
