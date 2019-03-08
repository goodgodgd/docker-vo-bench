import os
import os.path as op
import numpy as np
import pandas as pd
import matplotlib.backends.backend_qt5agg
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

import settings
from define_paths import *
import evaluation.eval_common as ec


def plot_dataset(dataset, save_path, ate_limit, rpe_limit, rae_limit):
    matplotlib.rcParams.update({'font.size': 8})
    fig = plt.figure(figsize=(15, 4))
    eval_path = op.join(OUTPUT_PATH, "eval_result")

    ax = fig.add_subplot(131)
    filename = op.join(eval_path, "ate", dataset, "collect_te_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8")
    draw_error_plot(ax, errors, "ATE", ate_limit)

    ax = fig.add_subplot(132)
    filename = op.join(eval_path, "rpe", dataset, "collect_te_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8")
    draw_error_plot(ax, errors, "RPE", rpe_limit)

    ax = fig.add_subplot(133)
    filename = op.join(eval_path, "rpe", dataset, "collect_re_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8")
    draw_error_plot(ax, errors, "RAE", rae_limit)

    save_name = op.join(save_path, f"{dataset}.png")
    show_and_save(save_name)


def draw_error_plot(ax, data, ylabel, ylimit):
    colormap = matplotlib.cm.get_cmap('tab20', len(ec.ALGORITHMS))
    styles = ['-', '--', '-.', ':', '.', '+', 'x', '1']
    for i, column in enumerate(ec.ALGORITHMS):
        if column not in data.columns:
            continue
        coldata = data[column].values
        coldata = np.sort(coldata)
        coldata = coldata[~np.isnan(coldata)]
        linestyle = styles[i % len(styles)]
        ax.plot(list(range(len(coldata))), coldata,
                linestyle, color=colormap(i), label=column)

    ax.legend()
    ax.set_xlabel('runs')
    ax.set_ylabel(ylabel)
    x1, x2, y1, y2 = plt.axis()
    plt.axis([x1, x2, 0, ylimit])


def show_and_save(savename):
    print("savename:", savename)
    plt.pause(0.1)
    plt.savefig(savename, dpi=100)
    plt.waitforbuttonpress(1)
    plt.clf()


def main():
    assert op.isdir(OUTPUT_PATH)
    savepath = op.join(OUTPUT_PATH, "eval_result", "figures")
    os.makedirs(savepath, exist_ok=True)
    plot_dataset("euroc_mav", savepath, 2, 2, 0.4)
    plot_dataset("tum_vi", savepath, 5, 2, 0.4)
    # plt.show(block=True)


if __name__ == "__main__":
    main()
