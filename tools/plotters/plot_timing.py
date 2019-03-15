import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import matplotlib.backends.backend_qt5agg
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

import settings
from define_paths import *
import evaluation.eval_common as ec


def boxplot_timings(dataset, save_path):
    estim_path = op.join(OUTPUT_PATH, "pose_body", dataset)
    assert op.isdir(estim_path), "No pose output directory: " + estim_path

    if dataset.startswith("euroc"):
        sequence = "MH01"
    elif dataset.startswith("tum"):
        sequence = "corridor1"
    else:
        return
    test_id = 0
    raw_timings = {}

    for algo_name in ec.ALGORITHMS:
        estim_file = "{}_{}_{}.txt".format(algo_name, sequence, test_id)
        estim_file = op.join(estim_path, estim_file)
        if not op.isfile(estim_file):
            continue
        data = np.loadtxt(estim_file)
        if data.shape[1] < 9:
            continue
        raw_timings[algo_name] = data[:, 8]
        print("timing length", estim_file, raw_timings[algo_name].shape)

    # draw boxplot here


def main():
    assert op.isdir(OUTPUT_PATH)
    savepath = op.join(OUTPUT_PATH, "eval_result", "figures")
    os.makedirs(savepath, exist_ok=True)
    boxplot_timings("euroc_mav", savepath)
    boxplot_timings("tum_vi", savepath)
    # plt.show(block=True)


if __name__ == "__main__":
    main()
