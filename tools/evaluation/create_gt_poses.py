import os
import os.path as op
import pandas as pd
import glob
import numpy as np

import settings
from define_paths import *
import sequence_abbrev as sa
import evaluation.rotation as rotation

# Note: Set EUROC_PATH, OUTPUT_PATH in defins_paths.py before run this


def create_gt_poses(srcpath, dstpath, dataset, data_reader):
    seq_paths, result_path = preprocess(srcpath, dstpath, dataset)
    os.makedirs(result_path, exist_ok=True)
    print("===== create gtposes to", result_path)

    for seq_path in seq_paths:
        print("read sequence:", seq_path)
        data = data_reader(seq_path)
        seq_abbr = sa.sequence_abbrev(dataset, op.basename(seq_path))
        savefile = op.join(result_path, seq_abbr + ".csv")
        process_and_save(data, savefile, "%1.09f")


def preprocess(srcpath, dstpath, dataset):
    if dataset.startswith("euroc") or dataset.startswith("tum"):
        seq_paths = [s.rstrip("/") for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    else:
        raise NotImplementedError()

    result_path = op.join(dstpath, "ground_truth", dataset)
    return seq_paths, result_path


def read_euroc(seq_path):
    data = pd.read_csv(op.join(seq_path, "mav0/state_groundtruth_estimate0/data.csv"))
    # extract required columns: timestamp, x, y, z, qx, qy, qz, qw
    req_columns = ['#timestamp', ' p_RS_R_x [m]', ' p_RS_R_y [m]', ' p_RS_R_z [m]',
                   ' q_RS_x []', ' q_RS_y []', ' q_RS_z []', ' q_RS_w []']
    data = data[req_columns]
    # rename columns
    new_names = ["#timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    renamer = dict(zip(req_columns, new_names))
    data = data.rename(columns=renamer)
    return data


def read_tumvi(seq_path):
    data = pd.read_csv(op.join(seq_path, "mav0/mocap0/data.csv"))
    reorder_cols = ['#timestamp [ns]', ' p_RS_R_x [m]', ' p_RS_R_y [m]', ' p_RS_R_z [m]',
                    ' q_RS_x []', ' q_RS_y []', ' q_RS_z []', ' q_RS_w []']
    data = data[reorder_cols]
    # rename columns
    new_names = ["#timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    renamer = dict(zip(list(data), new_names))
    data = data.rename(columns=renamer)
    return data


def process_and_save(posedf, savefile, float_format):
    # nanosecond timestamp to second
    posedf["#timestamp"] = posedf["#timestamp"] / 1.0E+9
    posedf = posedf.sort_values(by="#timestamp")

    # divide by first pose for trajectory start from the zero pose
    first_pose_mat = rotation.transform44(posedf.iloc[0, :].values)
    for index in posedf.index:
        timestamp = posedf.loc[index, "#timestamp"]
        gt_pose_mat = rotation.transform44(posedf.loc[index, :].values)
        zerobase_mat = np.dot(np.linalg.inv(first_pose_mat), gt_pose_mat)
        posedf.loc[index, :] = rotation.pose_quat(zerobase_mat, timestamp)

    posedf = posedf.set_index("#timestamp")
    # save file
    print("save to", savefile)
    print(posedf.head())
    posedf.to_csv(savefile, encoding="utf-8", float_format=float_format)


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    assert op.isdir(EUROC_PATH), "No dir: " + EUROC_PATH
    assert op.isdir(TUMVI_PATH), "No dir: " + TUMVI_PATH
    assert op.isdir(OUTPUT_PATH), "No dir: " + OUTPUT_PATH

    create_gt_poses(EUROC_PATH, OUTPUT_PATH, "euroc_mav", read_euroc)
    create_gt_poses(TUMVI_PATH, OUTPUT_PATH, "tum_vi", read_tumvi)
