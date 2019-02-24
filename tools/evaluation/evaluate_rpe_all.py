import os
import os.path as op
import numpy as np
import pandas as pd
import glob

import settings
from define_paths import *
import evaluation.evaluate_rpe as rpe

ALGORITHMS = ["orb2_vo_stereo", "rovioli_mvio",
              "vinsfs_mvio", "vinsfs_svio", "vinsfs_stereo",
              "svo2_mvio", "svo2_svio", "svo2_stereo"]
NUM_TEST = 2
MAX_TIME_DIFF = 0.5


def evaluate_rpe_all(dataset):
    estim_path = op.join(OUTPUT_PATH, dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    result_path = op.join(OUTPUT_PATH, "eval_files", "rpe", dataset)
    assert op.isdir(estim_path), "No pose output directory: " + estim_path
    assert op.isdir(gtruth_path), "No ground truth directory: " + gtruth_path
    os.makedirs(result_path, exist_ok=True)

    sequences = list_sequences(gtruth_path)

    statis_results = {}
    rawerr_results = {}
    for algo_name in ALGORITHMS:
        stat_result = []
        raw_result = []
        print("\n===== algorithm: {} =====".format(algo_name))

        for seq_name in sequences:
            for test_id in range(NUM_TEST):
                gtruth_file = op.join(gtruth_path, seq_name + ".csv")
                estim_file = "{}_{}_{}.txt".format(algo_name, seq_name, test_id)
                estim_file = op.join(estim_path, estim_file)
                if not op.isfile(estim_file):
                    continue

                print("gt, est file", op.basename(gtruth_file), op.basename(estim_file))
                traj_gt = rpe.read_trajectory(gtruth_file)
                traj_est = rpe.read_trajectory(estim_file)
                # for time, pose in traj_est.items():
                #     traj_est[time] = np.linalg.inv(pose)
                traj_est = remove_zero_frames(traj_est)

                print("compute rpe", algo_name, seq_name, test_id)
                rpe_result = compute_rpe(traj_gt, traj_est, 10)
                stats = calc_statistics(rpe_result, list(traj_gt.keys()))
                seq_result = [seq_name, test_id, *stats]
                stat_result.append(seq_result)
                raw_result.append(rpe_result[["te", "re"]].values)
        
        if stat_result:
            statis_results[algo_name] = pd.DataFrame(data=stat_result, columns=get_column_names())
            printcols = ["sequence", "testid", "te_mean", "re_mean", "track_seconds"]
            print(statis_results[algo_name].loc[:, printcols])
            rawerr_results[algo_name] = np.concatenate(raw_result, axis=0)
            print("raw errors shape:", rawerr_results[algo_name].shape)

    save_results(statis_results, rawerr_results, result_path)
    collect_fields_and_save(statis_results, ["te_mean", "re_mean", "track_ratio"], result_path)


def list_sequences(gtruth_path):
    gtfiles = glob.glob(gtruth_path + "/*")
    seq_abbr = [abbr.split("/")[-1][:-4] for abbr in gtfiles]
    seq_abbr.sort()
    print("sequences:", seq_abbr)
    return seq_abbr


def compute_rpe(traj_gt, traj_est, time_delta):
    rpe_result = rpe.evaluate_trajectory(traj_gt, traj_est,
                                         param_max_pairs=0,
                                         param_fixed_delta=True,
                                         param_delta=time_delta,
                                         param_delta_unit="s",
                                         param_offset=0.00,
                                         param_scale=1.00)
    rpe_result = np.array(rpe_result)
    columns = ["time_est_0", "time_est_1", "time_gt_0", "time_gt_1", "te", "re"]
    rpe_result = pd.DataFrame(data=rpe_result, columns=columns)
    return rpe_result


def remove_zero_frames(traj_est):
    timestamps = list(traj_est.keys())
    timestamps.sort()

    # remove default poses
    for time in timestamps:
        pose = traj_est[time]
        posit_sq = np.linalg.norm(pose[0, 0:3])
        if posit_sq < 1.0E-9:
            # print("remove zero", pose)
            del traj_est[time]

    # remove stopping poses
    for time, time_bef in zip(timestamps[1:], timestamps[:-1]):
        if time in traj_est.keys() and time_bef in traj_est.keys():
            pose = traj_est[time]
            pose_bef = traj_est[time_bef]
            posit_diff = (pose[:3, 3] - pose_bef[:3, 3]).T
            move = np.linalg.norm(posit_diff)
            if move < 1.0E-9:
                # print("remove stopping frame", posit_diff)
                del traj_est[time]

    return traj_est


def calc_statistics(rpe_result, gt_times):
    # table_columns = ["sequence", "testid",
    #                  "te_mean", "te_std", "te_min", "te_max", "te_med",
    #                  "re_mean", "re_std", "re_min", "re_max", "re_med",
    #                  "total_seconds", "track_seconds", "track_ratio"]
    te = rpe_result["te"].values
    re = rpe_result["re"].values
    stats = [np.mean(te), np.std(te), np.min(te), np.max(te), np.median(te), 
             np.mean(re), np.std(re), np.min(re), np.max(re), np.median(re)]

    total_seconds = accumulate_connected_time(gt_times, max_diff=MAX_TIME_DIFF)
    track_seconds = accumulate_connected_time(rpe_result["time_est_0"], max_diff=MAX_TIME_DIFF)
    track_ratio = track_seconds / total_seconds
    stats.extend([total_seconds, track_seconds, track_ratio])
    return stats


def accumulate_connected_time(timestamps, max_diff):
    time_diff = np.array(timestamps[1:]) - np.array(timestamps[:-1])
    time_diff = time_diff[time_diff < max_diff]
    return np.sum(time_diff)


def get_column_names():
    return ["sequence", "testid",
            "te_mean", "te_std", "te_min", "te_max", "te_med",
            "re_mean", "re_std", "re_min", "re_max", "re_med", 
            "total_seconds", "track_seconds", "track_ratio"]


def save_results(total_results, total_errors, save_path):
    for algo_name, result in total_results.items():
        filename = op.join(save_path, "rpe_{}_summary.csv".format(algo_name))
        print("save", op.basename(filename))
        result.to_csv(filename, encoding="utf-8", float_format="%1.6f")

    for algo_name, result in total_errors.items():
        filename = op.join(save_path, "rpe_{}_error.txt".format(algo_name))
        print("save", op.basename(filename))
        np.savetxt(filename, result, fmt="%1.6f")


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
        filename = op.join(save_path, "rpe_{}.csv".format(fname))
        print("save", op.basename(filename))
        field_data_merged.to_csv(filename, encoding="utf-8", float_format="%1.6f")


def main():
    np.set_printoptions(precision=5, suppress=True)
    evaluate_rpe_all("euroc_mav")


if __name__ == "__main__":
    main()
