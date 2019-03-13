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
    eval_path = op.join(OUTPUT_PATH, "eval_result")

    if dataset.startswith("euroc"):
        fignum = 0
        figsize = (8, 6)
        envnames = ["MH", "V1", "V2"]
        plot_layout = 22
    elif dataset.startswith("tum"):
        fignum = 1
        figsize = (8, 9)
        envnames = ["corridor", "magistrale", "outdoors", "room", "slides"]
        plot_layout = 32
    else:
        return

    fig = plt.figure(num=fignum, figsize=figsize)
    draw_ate(fig, eval_path, dataset, envnames, plot_layout, ate_limit)
    save_name = op.join(save_path, f"{dataset}_ate.png")
    show_and_save(save_name)

    draw_rpte(fig, eval_path, dataset, envnames, plot_layout, rpe_limit)
    save_name = op.join(save_path, f"{dataset}_rpte.png")
    show_and_save(save_name)

    draw_rpre(fig, eval_path, dataset, envnames, plot_layout, rae_limit)
    save_name = op.join(save_path, f"{dataset}_rpre.png")
    show_and_save(save_name)


def draw_ate(fig, eval_path, dataset, envnames, plot_layout, ate_limit):
    filename = op.join(eval_path, "ate", dataset, "collect_te_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8", index_col=False)
    errors = errors.drop(columns="Unnamed: 0")

    ax = fig.add_subplot(plot_layout*10 + 1)
    draw_error_plot(ax, errors, "ATE_total", ate_limit)

    for i, env in enumerate(envnames):
        ax = fig.add_subplot(plot_layout*10 + i + 2)
        env_errors = errors[errors["sequence"].str.contains(env)]
        draw_error_plot(ax, env_errors, "ATE_" + env, ate_limit)


def draw_rpte(fig, eval_path, dataset, envnames, plot_layout, ate_limit):
    filename = op.join(eval_path, "rpe", dataset, "collect_te_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8", index_col=False)
    errors = errors.drop(columns="Unnamed: 0")

    ax = fig.add_subplot(plot_layout*10 + 1)
    draw_error_plot(ax, errors, "RPTE_total", ate_limit)

    for i, env in enumerate(envnames):
        ax = fig.add_subplot(plot_layout*10 + i + 2)
        env_errors = errors[errors["sequence"].str.contains(env)]
        draw_error_plot(ax, env_errors, "RPTE_" + env, ate_limit)


def draw_rpre(fig, eval_path, dataset, envnames, plot_layout, ate_limit):
    filename = op.join(eval_path, "rpe", dataset, "collect_re_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8", index_col=False)
    errors = errors.drop(columns="Unnamed: 0")

    ax = fig.add_subplot(plot_layout*10 + 1)
    draw_error_plot(ax, errors, "RPRE_total", ate_limit)

    for i, env in enumerate(envnames):
        ax = fig.add_subplot(plot_layout*10 + i + 2)
        env_errors = errors[errors["sequence"].str.contains(env)]
        draw_error_plot(ax, env_errors, "RPRE_" + env, ate_limit)


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
    plt.waitforbuttonpress()
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
