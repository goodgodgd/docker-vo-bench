import os
import os.path as op
import numpy as np
import pandas as pd
import glob

import settings

ALGORITHMS = ["rovioli_mvio", "orb2_vo_stereo",
              "vinsfs_mvio", "vinsfs_svio", "vinsfs_stereo",
              "svo2_mvio", "svo2_svio", "svo2_stereo"]
MAX_TIME_DIFF = 0.5


def clear_files(tgtpath):
    print("clear dir", tgtpath)
    files = [op.join(tgtpath, item) for item in os.listdir(tgtpath)]
    for file in files:
        os.remove(file)


def list_sequences(gtruth_path):
    gtfiles = glob.glob(gtruth_path + "/*")
    seq_abbr = [abbr.split("/")[-1][:-4] for abbr in gtfiles]
    seq_abbr.sort()
    print("sequences:", seq_abbr)
    return seq_abbr


def check_tracking_time(traj_gt, traj_est):
    gt_times = np.sort(np.array(list(traj_gt.keys())))
    est_times = np.sort(np.array(list(traj_est.keys())))
    gt_total = gt_times[-1] - gt_times[0]
    est_total = accumulate_connected_time(est_times, max_diff=MAX_TIME_DIFF)
    return est_total / gt_total


def save_results(total_results, total_errors, save_path):
    for algo_name, result in total_results.items():
        filename = op.join(save_path, "summary_{}.csv".format(algo_name))
        print("save", op.basename(filename))
        result.to_csv(filename, encoding="utf-8", float_format="%1.6f")

    # for algo_name, result in total_errors.items():
    #     filename = op.join(save_path, "rawerr_{}.txt".format(algo_name))
    #     print("save", op.basename(filename))
    #     np.savetxt(filename, result, fmt="%1.6f")


def collect_fields_and_save(statis_results, fields, save_path):
    for fname in fields:
        columns = []
        field_data_merged = None
        for algo_name, result in statis_results.items():
            columns.append(algo_name)
            field_data = result[["sequence", "testid", fname]]
            field_data = field_data.rename(columns={fname: algo_name})
            # print("field data\n", field_data)

            if field_data_merged is None:
                field_data_merged = field_data
            else:
                field_data_merged = pd.merge(field_data_merged, field_data,
                                             on=["sequence", "testid"], how="outer")

        # print("merged field data\n", field_data_merged)
        filename = op.join(save_path, "collect_{}.csv".format(fname))
        print("save", op.basename(filename))
        field_data_merged.to_csv(filename, encoding="utf-8", float_format="%1.6f")


def accumulate_connected_time(timestamps, max_diff):
    time_diff = np.array(timestamps[1:]) - np.array(timestamps[:-1])
    time_diff = time_diff[time_diff < max_diff]
    return np.sum(time_diff)
