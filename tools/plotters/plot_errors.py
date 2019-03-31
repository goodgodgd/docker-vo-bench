import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import matplotlib.backends.backend_qt5agg
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

import settings
from define_paths import *
import evaluation.eval_common as ec


def plot_dataset(dataset, save_path, ate_limit, rpte_limit, rpre_limit):
    matplotlib.rcParams.update({'font.size': 10})
    eval_path = op.join(OUTPUT_PATH, "eval_result")

    if dataset.startswith("euroc"):
        fignum, figsize, plot_layout, categories = get_euroc_params()
    elif dataset.startswith("tum"):
        fignum, figsize, plot_layout, categories = get_tumvi_params()
    else:
        return

    fig = plt.figure(num=fignum, figsize=figsize)
    fig.set_size_inches(figsize[0], figsize[1], forward=True)
    algorithms = draw_ate(fig, eval_path, dataset, categories, plot_layout, ate_limit)
    save_name = op.join(save_path, f"{dataset}_ate.png")
    show_and_save(save_name)

    draw_rpe(fig, eval_path, dataset, plot_layout, rpte_limit, rpre_limit)
    save_name = op.join(save_path, f"{dataset}_rpe.png")
    show_and_save(save_name)

    draw_label(fig, plot_layout, algorithms)
    save_name = op.join(save_path, f"{dataset}_label.png")
    show_and_save(save_name)


def get_euroc_params():
    fignum = 0
    figsize = (8, 6)
    plot_layout = 22
    categories = {"easy": ["MH01", "MH02", "V101", "V201"],
                  "medium": ["MH03", "V102", "V202"],
                  "diffcult": ["MH04", "MH05", "V103", "V203"]}
    return fignum, figsize, plot_layout, categories


def get_tumvi_params():
    fignum = 0
    figsize = (8, 9)
    plot_layout = 32
    tumvi_gtpath = op.join(OUTPUT_PATH, "ground_truth", "tum_vi/")
    envnames = ["corridor", "magistrale", "outdoors", "room", "slides"]
    categories = {}
    for env in envnames:
        files = glob.glob(tumvi_gtpath + env + "*")
        sequences = []
        for file in files:
            seqname = file.replace(tumvi_gtpath, "").replace(".csv", "")
            sequences.append(op.basename(seqname))
        sequences.sort()
        categories[env] = sequences
    return fignum, figsize, plot_layout, categories


def draw_ate(fig, eval_path, dataset, categories, plot_layout, ate_limit):
    filename = op.join(eval_path, "ate", dataset, "collect_te_mean.csv")
    errors = pd.read_csv(filename, encoding="utf-8", index_col=False)
    errors = errors.drop(columns="Unnamed: 0")
    ax = fig.add_subplot(plot_layout*10 + 1)
    draw_error_plot(ax, errors, "ATE total (m)", ate_limit)

    for i, (tag, sequences) in enumerate(categories.items()):
        ax = fig.add_subplot(plot_layout*10 + i + 2)
        env_errors = errors[errors["sequence"].isin(sequences)]
        draw_error_plot(ax, env_errors, f"ATE {tag} (m)", ate_limit)

    plt.subplots_adjust(wspace=0.3, hspace=0.3)
    return list(errors)[2:]


def draw_rpe(fig, eval_path, dataset, plot_layout, rpte_limit, rpre_limit):
    filename = op.join(eval_path, "rpe", dataset, "collect_te_mean.csv")
    load_and_draw(fig, filename, plot_layout*10 + 1, rpte_limit, "RPTE mean (m)")

    filename = op.join(eval_path, "rpe", dataset, "collect_re_mean.csv")
    load_and_draw(fig, filename, plot_layout*10 + 2, rpre_limit, "RPRE mean (rad)")

    filename = op.join(eval_path, "rpe", dataset, "collect_te_max.csv")
    load_and_draw(fig, filename, plot_layout*10 + 3, rpte_limit, "RPTE max (m)")

    filename = op.join(eval_path, "rpe", dataset, "collect_re_max.csv")
    load_and_draw(fig, filename, plot_layout*10 + 4, rpre_limit, "RPRE max (rad)")


def load_and_draw(fig, filename, subplot_num, rpte_limit, vertical_tag):
    errors = pd.read_csv(filename, encoding="utf-8", index_col=False)
    errors = errors.drop(columns="Unnamed: 0")
    ax = fig.add_subplot(subplot_num)
    draw_error_plot(ax, errors, vertical_tag, rpte_limit)


def draw_error_plot(ax, data, ylabel, ylimit):
    colormap = matplotlib.cm.get_cmap('tab20', len(ec.ALGORITHMS))
    styles = ['-', '--', '-.', ':', '.', '+', 'x', '1']
    algcols = list(data.columns)[2:]
    for i, column in enumerate(algcols):
        coldata = data[column].values
        coldata = np.sort(coldata)
        coldata = coldata[~np.isnan(coldata)]
        linestyle = styles[i % len(styles)]
        ax.plot(list(range(len(coldata))), coldata,
                linestyle, color=colormap(i), label=column)

    num_runs = len(data["sequence"].unique())*5
    ax.set_xlabel('runs')
    ax.set_ylabel(ylabel)
    plt.axis([-1, num_runs, 0, ylimit])

    interval = num_runs // 5
    interval = (interval // 10) * 10 if interval > 10 else interval
    ax.xaxis.set_ticks(np.arange(0, num_runs + interval, interval))
    ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%1.0f'))
    interval = round(ylimit / 5, 1)
    ax.yaxis.set_ticks(np.arange(0, ylimit + interval, interval))
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%1.1f'))


def show_and_save(savename):
    plt.pause(0.1)
    plt.savefig(savename, dpi=100)
    print("savename:", savename)
    plt.waitforbuttonpress()
    plt.clf()


def draw_label(fig, plot_layout, algorithms):
    ax = fig.add_subplot(plot_layout*10 + 1)

    colormap = matplotlib.cm.get_cmap('tab20', len(ec.ALGORITHMS))
    styles = ['-', '--', '-.', ':', '.', '+', 'x', '1']
    for i, column in enumerate(algorithms):
        linestyle = styles[i % len(styles)]
        ax.plot([0, 1], [0, 0], linestyle, color=colormap(i), label=column)
    ax.legend()
    plt.axis([0, 10, 0, 10])


def main():
    assert op.isdir(OUTPUT_PATH)
    savepath = op.join(OUTPUT_PATH, "eval_result", "figures")
    os.makedirs(savepath, exist_ok=True)
    plot_dataset("euroc_mav", savepath, 1.5, 1.5, 0.4)
    plot_dataset("tum_vi", savepath, 5, 1.5, 0.4)
    # plt.show(block=True)


if __name__ == "__main__":
    main()
