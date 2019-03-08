import os
import os.path as op
import numpy as np
import pandas as pd
import glob

import settings
from define_paths import *
import evaluation.evaluate_rpe as rpe
import evaluation.eval_common as ec

NUM_TEST = 5


def evaluate_rpe_all(dataset):
    estim_path = op.join(OUTPUT_PATH, "pose_body", dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    result_path = op.join(OUTPUT_PATH, "eval_result", "rpe", dataset)
    assert op.isdir(estim_path), "No pose output directory: " + estim_path
    assert op.isdir(gtruth_path), "No ground truth directory: " + gtbody_path
    os.makedirs(result_path, exist_ok=True)
    ec.clear_files(result_path)

    sequences = ec.list_sequences(gtruth_path)

    statis_results = {}
    rawerr_results = {}
    for algo_name in ec.ALGORITHMS:
        stat_result = []
        raw_result = []
        print("\n===== dataset: {}, algorithm: {} =====".format(dataset, algo_name))

        for seq_name in sequences:
            for test_id in range(NUM_TEST):
                gtruth_file = op.join(gtruth_path, seq_name + ".csv")
                estim_file = "{}_{}_{}.txt".format(algo_name, seq_name, test_id)
                estim_file = op.join(estim_path, estim_file)
                if not op.isfile(estim_file):
                    continue

                print("gt, est file:", op.basename(gtruth_file), op.basename(estim_file))
                traj_gt = rpe.read_trajectory(gtruth_file)
                traj_est = rpe.read_trajectory(estim_file)
                track_ratio = ec.check_tracking_time(traj_gt, traj_est)
                if track_ratio < 0.5:
                    print("tracking time ratio is < 0.5, abandon this result")
                    continue

                # main function
                try:
                    rpe_result = compute_rpe(traj_gt, traj_est, 10)
                except ValueError:
                    print("skip this sequence")
                    continue

                stats = calc_statistics(rpe_result, list(traj_gt.keys()))
                seq_result = [seq_name, test_id, *stats]
                stat_result.append(seq_result)
                raw_result.append(rpe_result[["te", "re"]].values)
        
        if stat_result:
            statis_results[algo_name] = pd.DataFrame(data=stat_result, columns=get_column_names())
            printcols = ["sequence", "testid", "te_mean", "re_mean", "track_seconds"]
            print(algo_name + "\n", statis_results[algo_name].loc[:, printcols])
            rawerr_results[algo_name] = np.concatenate(raw_result, axis=0)
            print("raw errors shape:", rawerr_results[algo_name].shape)

    ec.save_results(statis_results, rawerr_results, result_path)
    ec.collect_fields_and_save(statis_results, ["te_mean", "re_mean", "track_ratio"], result_path)


def compute_rpe(traj_gt, traj_est, time_delta):
    rpe_result = rpe.evaluate_trajectory(traj_gt, traj_est,
                                         param_max_pairs=0,
                                         param_fixed_delta=True,
                                         param_delta=time_delta,
                                         param_delta_unit="s",
                                         param_offset=0.00,
                                         param_scale=1.00)
    rpe_result = np.array(rpe_result)
    rpe_result = remove_1percent(rpe_result, 4)
    if np.mean(rpe_result[:, 4]) > 100:
        print("translational error is too large", np.mean(rpe_result[:, 4]))
        raise ValueError()

    columns = ["time_est_0", "time_est_1", "time_gt_0", "time_gt_1", "te", "re"]
    rpe_result = pd.DataFrame(data=rpe_result, columns=columns)
    return rpe_result


def remove_1percent(rpe, column: int):
    error_sorted = np.sort(rpe[:, column], axis=None)
    length = len(error_sorted)
    thresh = error_sorted[int((length-1) * 0.99)]
    rpe = rpe[rpe[:, column] < thresh, :]
    return rpe


def calc_statistics(rpe_result, gt_times):
    # table_columns = ["sequence", "testid",
    #                  "te_mean", "te_std", "te_min", "te_max", "te_med",
    #                  "re_mean", "re_std", "re_min", "re_max", "re_med",
    #                  "total_seconds", "track_seconds", "track_ratio"]
    te = rpe_result["te"].values
    re = rpe_result["re"].values
    stats = [np.mean(te), np.std(te), np.min(te), np.max(te), np.median(te), 
             np.mean(re), np.std(re), np.min(re), np.max(re), np.median(re)]

    total_seconds = ec.accumulate_connected_time(gt_times, max_diff=ec.MAX_TIME_DIFF)
    track_seconds = ec.accumulate_connected_time(rpe_result["time_est_0"], max_diff=ec.MAX_TIME_DIFF)
    track_ratio = track_seconds / total_seconds
    stats.extend([total_seconds, track_seconds, track_ratio])
    return stats


def get_column_names():
    return ["sequence", "testid",
            "te_mean", "te_std", "te_min", "te_max", "te_med",
            "re_mean", "re_std", "re_min", "re_max", "re_med", 
            "total_seconds", "track_seconds", "track_ratio"]


def main():
    np.set_printoptions(precision=5, suppress=True)
    evaluate_rpe_all("euroc_mav")
    evaluate_rpe_all("tum_vi")


if __name__ == "__main__":
    main()
