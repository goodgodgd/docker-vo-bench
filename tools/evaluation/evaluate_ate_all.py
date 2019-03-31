import os
import os.path as op
import numpy as np
import pandas as pd

import settings
from define_paths import *
import evaluation.evaluate_ate as ate
import evaluation.eval_common as ec
import evaluation.associate as assoc

NUM_TEST = 5
MAX_TIME_DIFF = 0.5


def evaluate_ate_all(dataset):
    estim_path = op.join(OUTPUT_PATH, "pose_body", dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    result_path = op.join(OUTPUT_PATH, "eval_result", "ate", dataset)
    assert op.isdir(estim_path), "No pose output directory: " + estim_path
    assert op.isdir(gtruth_path), "No ground truth directory: " + gtruth_path
    os.makedirs(result_path, exist_ok=True)
    ec.clear_files(result_path)
    major_axes = "yz" if dataset.startswith("euroc") else "xy"

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
                print("sequence: {}, testid: {}".format(seq_name, test_id))
                traj_gt = assoc.read_file_list(gtruth_file)
                traj_est = assoc.read_file_list(estim_file)
                traj_est = remove_zero_frames(traj_est)
                track_ratio = ec.check_tracking_time(traj_gt, traj_est)
                if track_ratio < 0.5:
                    print("tracking time ratio is < 0.5, abandon this result")
                    continue

                try:    # throws error when track_ratio < 0.5
                    tran_errs, association = compute_ate(traj_gt, traj_est, estim_file,
                                                         result_path, major_axes)

                    stats = calc_statistics(tran_errs, association, list(traj_gt.keys()))

                    seq_result = [seq_name, test_id, *stats]
                    stat_result.append(seq_result)
                    raw_result.append(tran_errs)
                except ValueError:
                    print("something went wrong, result of this sequence is not saved")

        if stat_result:
            statis_results[algo_name] = pd.DataFrame(data=stat_result,
                                                     columns=get_column_names())
            printcols = ["sequence", "testid", "te_mean", "track_seconds"]
            print(statis_results[algo_name].loc[:, printcols])
            rawerr_results[algo_name] = np.concatenate(raw_result, axis=0)
            print("raw errors shape:", rawerr_results[algo_name].shape)

    ec.save_results(statis_results, rawerr_results, result_path)
    ec.collect_fields_and_save(statis_results, ["te_mean", "track_ratio"], result_path)


def remove_zero_frames(traj_est):
    timestamps = list(traj_est.keys())
    timestamps.sort()

    # remove default poses
    for time in timestamps:
        pose = np.array(traj_est[time], dtype=np.float64)
        posit_sq = np.linalg.norm(pose[:3])
        if posit_sq < 1.0E-9:
            del traj_est[time]

    # remove stopping poses
    for time, time_bef in zip(timestamps[1:], timestamps[:-1]):
        if time in traj_est.keys() and time_bef in traj_est.keys():
            pose = np.array(traj_est[time], dtype=np.float64)
            pose_bef = np.array(traj_est[time_bef], dtype=np.float64)
            posit_diff = (pose[:3] - pose_bef[:3]).T
            move = np.linalg.norm(posit_diff)
            if move < 1.0E-9:
                del traj_est[time]

    return traj_est


def calc_statistics(tran_errs, association, gt_tstamps):
    # table_columns = ["sequence", "testid",
    #                  "te_mean", "te_std", "te_min", "te_max", "te_med",
    #                  "total_seconds", "track_seconds", "track_ratio"]
    te = tran_errs
    stats = [np.mean(te), np.std(te), np.min(te), np.max(te), np.median(te)]
    assert np.mean(te) < 101, "translational error {}".format(np.mean(te))

    est_tstamps = association[:, 4]
    total_seconds = ec.accumulate_connected_time(gt_tstamps, max_diff=MAX_TIME_DIFF)
    track_seconds = ec.accumulate_connected_time(est_tstamps, max_diff=MAX_TIME_DIFF)
    track_ratio = track_seconds / total_seconds
    stats.extend([total_seconds, track_seconds, track_ratio])
    return stats


def compute_ate(traj_gt, traj_est, estim_file, result_path, major_axes):
    asso_name = op.join(result_path, "asso_" + op.basename(estim_file))
    plot_name = op.join(result_path, "plot_" + op.basename(estim_file).replace(".txt", ".png"))
    align_rot, align_trn, trjerr, association = \
        ate.evaluate_ate(traj_gt, traj_est, save_associations=asso_name,
                         plot=plot_name, major_axes=major_axes, plot_3d=None)
    print("aligning transformation: \n{}".format(np.concatenate([align_rot, align_trn], axis=1)))
    return trjerr, association


def get_column_names():
    return ["sequence", "testid",
            "te_mean", "te_std", "te_min", "te_max", "te_med",
            "total_seconds", "track_seconds", "track_ratio"]


def main():
    np.set_printoptions(precision=5, suppress=True)
    evaluate_ate_all("euroc_mav")
    # evaluate_ate_all("tum_vi")


if __name__ == "__main__":
    main()
