import os.path as op
import numpy as np
import pandas as pd
import matplotlib.backends.backend_qt5agg
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

import settings
from define_paths import *
import evaluation.eval_common as ec


def plot_ate(dataset):
    result_path = op.join(OUTPUT_PATH, "eval_result", "ate", dataset)
    te_file = op.join(result_path, "collect_te_mean.csv")
    trans_err = pd.read_csv(te_file, encoding="utf-8")
    # re_file = op.join(result_path, "collect_re_mean.csv")
    # rotat_err = pd.read_csv(re_file, encoding="utf-8")

    fig = plt.figure()
    ax = fig.add_subplot(111)
    draw_error_plot(ax, trans_err)

    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.show(block=True)


def draw_error_plot(ax, data):
    for column in list(data):
        coldata = data[column].values
        coldata = np.sort(coldata)
        ax.plot(list(range(len(coldata))), coldata, linestyle="-", color="blue", label=column)


def main():
    np.set_printoptions(precision=5, suppress=True)
    plot_ate("euroc_mav")
    plot_ate("tum_vi")


if __name__ == "__main__":
    main()




