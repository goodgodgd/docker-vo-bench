import os
import os.path as op
import numpy as np
import pandas as pd
import glob

import settings
from define_paths import *
import evaluation.evaluate_ate as ate
import evaluation.eval_common as ec

NUM_TEST = 2
MAX_TIME_DIFF = 0.5


def evaluate_ate_all(dataset):
    estim_path = op.join(OUTPUT_PATH, "pose", dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset + "_body")
    result_path = op.join(OUTPUT_PATH, "eval_result", "ate", dataset)
    assert op.isdir(estim_path), "No pose output directory: " + estim_path
    assert op.isdir(gtruth_path), "No ground truth directory: " + gtruth_path
    os.makedirs(result_path, exist_ok=True)
    ec.clear_files(result_path)

    sequences = ec.list_sequences(gtruth_path)

    statis_results = {}
    rawerr_results = {}
    for algo_name in ec.ALGORITHMS:
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

                # main function
                tran_errs, association, gt_tstamps = \
                    compute_ate(gtruth_file, estim_file, result_path)

                stats = calc_statistics(tran_errs, association, gt_tstamps)
                seq_result = [seq_name, test_id, *stats]
                stat_result.append(seq_result)
                raw_result.append(tran_errs)

        if stat_result:
            statis_results[algo_name] = pd.DataFrame(data=stat_result,
                                                     columns=get_column_names())
            printcols = ["sequence", "testid", "te_mean", "track_seconds"]
            print(statis_results[algo_name].loc[:, printcols])
            rawerr_results[algo_name] = np.concatenate(raw_result, axis=0)
            print("raw errors shape:", rawerr_results[algo_name].shape)

    ec.save_results(statis_results, rawerr_results, result_path)
    ec.collect_fields_and_save(statis_results, ["te_mean", "track_ratio"], result_path)


def calc_statistics(tran_errs, association, gt_tstamps):
    # table_columns = ["sequence", "testid",
    #                  "te_mean", "te_std", "te_min", "te_max", "te_med",
    #                  "total_seconds", "track_seconds", "track_ratio"]
    te = tran_errs
    stats = [np.mean(te), np.std(te), np.min(te), np.max(te), np.median(te)]

    total_seconds = ec.accumulate_connected_time(gt_tstamps, max_diff=MAX_TIME_DIFF)
    track_seconds = ec.accumulate_connected_time(association[:, 3], max_diff=MAX_TIME_DIFF)
    track_ratio = track_seconds / total_seconds
    stats.extend([total_seconds, track_seconds, track_ratio])
    return stats


def compute_ate(gtruth_file, estim_file, result_path):
    asso_name = op.join(result_path, "asso_" + op.basename(estim_file))
    plot_name = op.join(result_path, "plot_" + op.basename(estim_file).replace(".txt", ".png"))
    align_rot, align_trn, trjerr, association, gt_tstamps = \
        ate.evaluate_ate(gtruth_file, estim_file, save_associations=asso_name, plot=plot_name)
    print("sequnce: {}, \nalign rotation\n{} \nalign translation: {}".
          format(op.basename(estim_file), align_rot, align_trn.T))
    return trjerr, association, gt_tstamps


def get_column_names():
    return ["sequence", "testid",
            "te_mean", "te_std", "te_min", "te_max", "te_med",
            "total_seconds", "track_seconds", "track_ratio"]


def main():
    np.set_printoptions(precision=5, suppress=True)
    evaluate_ate_all("euroc_mav")
    evaluate_ate_all("tum_vi")


if __name__ == "__main__":
    main()
