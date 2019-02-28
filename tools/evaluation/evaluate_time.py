import os
import os.path as op
import numpy as np
import pandas as pd

import settings
from define_paths import *
import evaluation.eval_common as ec

NUM_TEST = 2
MAX_TIME_DIFF = 0.5


def evaluate_ate_all(dataset):
    estim_path = op.join(OUTPUT_PATH, "pose", dataset)
    result_path = op.join(OUTPUT_PATH, "eval_result", "time", dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset + "_body")
    assert op.isdir(estim_path), "No pose output directory: " + estim_path
    os.makedirs(result_path, exist_ok=True)
    ec.clear_files(result_path)

    sequences = ec.list_sequences(gtruth_path)

    statis_results = {}
    for algo_name in ec.ALGORITHMS:
        stat_result = []
        print("\n===== dataset: {}, algorithm: {} =====".format(dataset, algo_name))

        for seq_name in sequences:
            for test_id in range(NUM_TEST):
                estim_file = "{}_{}_{}.txt".format(algo_name, seq_name, test_id)
                estim_file = op.join(estim_path, estim_file)
                if not op.isfile(estim_file):
                    continue
                print("sequence: {}, testid: {}".format(seq_name, test_id))

                # main function
                estim_pose = np.loadtxt(estim_file)
                if estim_pose.shape[1] != 9:
                    print("elapsed time was not recorded")
                    continue
                
                estim_time = estim_pose[:, 8]
                stats = calc_statistics(estim_time)
                seq_result = [seq_name, test_id, *stats]
                stat_result.append(seq_result)

        if stat_result:
            statis_results[algo_name] = pd.DataFrame(data=stat_result,
                                                     columns=get_column_names())
            printcols = ["sequence", "testid", "ft_mean", "ft_max"]
            print(statis_results[algo_name].loc[:, printcols])

    ec.save_results(statis_results, None, result_path)
    ec.collect_fields_and_save(statis_results, ["ft_mean", "ft_max"], result_path)


def calc_statistics(frame_times):
    ft = frame_times
    stats = [np.mean(ft), np.std(ft), np.min(ft), np.max(ft), np.median(ft)]
    return stats


def get_column_names():
    return ["sequence", "testid", "ft_mean", "ft_std", "ft_min", "ft_max", "ft_med"]


def main():
    np.set_printoptions(precision=5, suppress=True)
    evaluate_ate_all("euroc_mav")
    evaluate_ate_all("tum_vi")


if __name__ == "__main__":
    main()
