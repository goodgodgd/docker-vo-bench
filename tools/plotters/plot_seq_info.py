import os
import os.path as op
import pandas as pd
import matplotlib.backends.backend_qt5agg
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

import settings
from define_paths import *
import evaluation.eval_common as ec


def plot_seq_info(dataset, figpath):
    data = read_seq_info(dataset)
    speed_limit = 4 if dataset.startswith("euroc") else 5
    create_figure(data, speed_limit)
    save_figure(figpath, dataset)


def read_seq_info(dataset):
    seqfile = op.join(OUTPUT_PATH, "eval_result", "seq_info", dataset + ".csv")
    data = pd.read_csv(seqfile, encoding="utf-8")
    return data


def create_figure(data, speed_limit):
    matplotlib.rcParams.update({'font.size': 8})
    figsize = (5, 3)
    fig = plt.figure(num=0, figsize=figsize)
    fig.set_size_inches(figsize[0], figsize[1], forward=True)

    ax1 = fig.add_subplot(11 * 10 + 1)
    plot_velocities(ax1, data, speed_limit)
    ax2 = ax1.twinx()
    plot_time(ax2, data)


def plot_velocities(ax, data, speed_limit):
    columns = ["max tran", "mean tran", "max rota", "mean rota"]
    styles = ['.', '+', 'x', '1', '-', ':', '--', '-.']
    colormap = matplotlib.cm.get_cmap('tab20', len(ec.ALGORITHMS))
    color = 'tab:red'

    for i, column in enumerate(columns):
        linestyle = styles[i % len(styles)]
        ax.plot(list(range(len(data[column]))), data[column],
                linestyle, color=colormap(i), label=column)
        ax.tick_params(axis='y', labelcolor=color)

    ax.plot([0, 1], [-1, -1], ":", color=colormap(5), label="seq time")
    ax.legend()
    ax.set_xlabel('sequences')
    ax.set_ylabel('v(m/s), w(rad/s)', color=color)
    x1, x2, y1, y2 = plt.axis()
    plt.axis([x1, x2, 0, speed_limit])


def plot_time(ax, data):
    colormap = matplotlib.cm.get_cmap('tab20', len(ec.ALGORITHMS))
    color = 'tab:blue'
    times = data["total_time"].tolist()
    maxtime = (data["total_time"].max() // 100 + 1) * 100
    ax.plot(list(range(len(times))), times, ":", color=colormap(5))
    ax.tick_params(axis='y', labelcolor=color)
    ax.set_ylabel('seq time', color=color)
    x1, x2, y1, y2 = plt.axis()
    plt.axis([x1, x2, 0, maxtime])


def save_figure(figpath, dataset):
    savename = op.join(figpath, "seqinfo_" + dataset + ".png")
    plt.pause(0.1)
    plt.savefig(savename, dpi=100)
    print("savename:", savename)
    plt.waitforbuttonpress()
    plt.clf()


def main():
    assert op.isdir(OUTPUT_PATH)
    savepath = op.join(OUTPUT_PATH, "eval_result", "figures")
    os.makedirs(savepath, exist_ok=True)
    plot_seq_info("euroc_mav", savepath)
    plot_seq_info("tum_vi", savepath)
    # plt.show(block=True)


if __name__ == "__main__":
    main()
