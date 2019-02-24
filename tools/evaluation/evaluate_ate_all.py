import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import argparse

import settings
from define_paths import *
import evaluation.evaluate_ate as ate

ALGORITHMS = ["orb2_vo_stereo", "rovioli_mvio",
              "vinsfs_mvio", "vinsfs_svio", "vinsfs_stereo",
              "svo2_mvio", "svo2_svio", "svo2_stereo"]
NUM_TEST = 2
MAX_TIME_DIFF = 0.5


def evaluate_ate_all(dataset):
    estim_path = op.join(OUTPUT_PATH, dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    result_path = op.join(OUTPUT_PATH, "eval_files", "ate", dataset)
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

                tran_errs, association, gt_tstamps = \
                    compute_ate(gtruth_file, estim_file, result_path)
                stats = calc_statistics(tran_errs, association, gt_tstamps)
                seq_result = [seq_name, test_id, *stats]
                stat_result.append(seq_result)
                raw_result.append(tran_errs)

        if stat_result:
            statis_results[algo_name] = pd.DataFrame(data=stat_result,
                                                     columns=get_column_names())
            printcols = ["sequence", "testid", "te_mean", "re_mean", "track_seconds"]
            print(statis_results[algo_name].loc[:, printcols])
            rawerr_results[algo_name] = np.concatenate(raw_result, axis=0)
            print("raw errors shape:", rawerr_results[algo_name].shape)

    save_results(statis_results, rawerr_results, result_path)
    collect_fields_and_save(statis_results, ["te_mean", "track_ratio"],
                            result_path)


def calc_statistics(tran_errs, association, gt_tstamps):
    # table_columns = ["sequence", "testid",
    #                  "te_mean", "te_std", "te_min", "te_max", "te_med",
    #                  "total_seconds", "track_seconds", "track_ratio"]
    te = tran_errs
    stats = [np.mean(te), np.std(te), np.min(te), np.max(te), np.median(te)]

    total_seconds = accumulate_connected_time(gt_tstamps, max_diff=MAX_TIME_DIFF)
    track_seconds = accumulate_connected_time(association[:, 3], max_diff=MAX_TIME_DIFF)
    track_ratio = track_seconds / total_seconds
    stats.extend([total_seconds, track_seconds, track_ratio])
    return stats


def list_sequences(gtruth_path):
    gtfiles = glob.glob(gtruth_path + "/*")
    seq_abbr = [abbr.split("/")[-1][:-4] for abbr in gtfiles]
    seq_abbr.sort()
    print(seq_abbr)
    return seq_abbr


def compute_ate(gtruth_file, estim_file, result_path):
    asso_name = op.join(result_path, "asso_" + op.basename(estim_file))
    plot_name = op.join(result_path, "plot_" + op.basename(estim_file).replace(".txt", ".png"))
    rot, trn, trjerr, association, gt_tstamps = \
        ate.evaluate_ate(gtruth_file, estim_file, save_associations=asso_name, plot=plot_name)
    print("sequnce: {}, \nrotation\n{} \ntranslation\n{}".
          format(op.basename(estim_file), rot, trn.T))
    return trjerr, association, gt_tstamps


def accumulate_connected_time(timestamps, max_diff):
    time_diff = np.array(timestamps[1:]) - np.array(timestamps[:-1])
    time_diff = time_diff[time_diff < max_diff]
    return np.sum(time_diff)


def get_column_names():
    return ["sequence", "testid",
            "te_mean", "te_std", "te_min", "te_max", "te_med",
            "total_seconds", "track_seconds", "track_ratio"]


def save_results(total_results, total_errors, save_path):
    for algo_name, result in total_results.items():
        filename = op.join(save_path, "ate_{}_summary.csv".format(algo_name))
        print("save", op.basename(filename))
        result.to_csv(filename, encoding="utf-8", float_format="%1.6f")

    for algo_name, result in total_errors.items():
        filename = op.join(save_path, "ate_{}_error.txt".format(algo_name))
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
        filename = op.join(save_path, "ate_{}.csv".format(fname))
        print("save", op.basename(filename))
        field_data_merged.to_csv(filename, encoding="utf-8", float_format="%1.6f")


def main():
    np.set_printoptions(precision=5, suppress=True)
    evaluate_ate_all("euroc_mav")


if __name__ == "__main__":
    main()
