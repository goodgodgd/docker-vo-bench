import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import copy
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

    raw_timings = {}
    algorithms = copy.deepcopy(ec.ALGORITHMS)
    algorithms.remove("rovioli_mvio")

    for algo_name in algorithms:
        estim_pattern = "{}/{}_{}_*.txt".format(estim_path, algo_name, sequence)
        estim_files = glob.glob(estim_pattern)
        if not estim_files:
            continue
        timings = []
        for file in estim_files:
            data = np.loadtxt(file)
            if data.shape[1] < 9:
                continue
            timings.append(data[:, 8])
        timings = np.concatenate(timings, 0)
        inds = np.linspace(0, len(timings), 1000, endpoint=False).astype(int)
        timings = timings[inds]
        raw_timings[algo_name] = timings
        print("timing length", estim_pattern, raw_timings[algo_name].shape)

    mean_time = {key: np.mean(value) for key, value in raw_timings.items()}
    print(mean_time)

    matplotlib.rcParams.update({'font.size': 8})
    fig = plt.figure(num=0, figsize=(7, 3))
    result = plt.boxplot(list(raw_timings.values()), labels=list(raw_timings.keys()), flierprops={"marker": '.'})
    x1, x2, y1, y2 = plt.axis()
    plt.axis([x1, x2, 0, 200])
    print("result of boxplot", result["fliers"])
    save_name = op.join(save_path, f"{dataset}_timing.png")
    show_and_save(save_name)


def show_and_save(savename):
    plt.pause(0.1)
    plt.savefig(savename, dpi=100)
    print("savename:", savename)
    plt.waitforbuttonpress()
    plt.clf()


def main():
    assert op.isdir(OUTPUT_PATH)
    savepath = op.join(OUTPUT_PATH, "eval_result", "figures")
    os.makedirs(savepath, exist_ok=True)
    boxplot_timings("euroc_mav", savepath)
    boxplot_timings("tum_vi", savepath)
    # plt.show(block=True)


if __name__ == "__main__":
    main()
