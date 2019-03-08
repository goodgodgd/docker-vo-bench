import os.path as op
import numpy as np
import pandas as pd
import matplotlib.backends.backend_qt5agg
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

from define_paths import *
import evaluation.eval_common as ec


def draw_trajectory(dataset):
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    sequences = ec.list_sequences(gtruth_path)
    fig = plt.figure()
    ax = Axes3D(fig)

    for seq_name in sequences:
        print("sequence:", seq_name)
        gtruth_file = op.join(gtruth_path, seq_name + ".csv")
        trajectory = pd.read_csv(gtruth_file)
        trajectory = trajectory.values
        timestamp = trajectory[:, 0]
        traj_xyz = trajectory[:, 1:4]
        extent = np.array([np.min(traj_xyz, axis=0), np.max(traj_xyz, axis=0),
                          np.max(traj_xyz, axis=0) - np.min(traj_xyz, axis=0)])
        print("min, max, max-min\n", extent)
        plot3d(ax, timestamp, traj_xyz)

    plt.show(block=True)


def plot3d(ax, timestamp, traj_xyz):
    plot_traj3d(ax, timestamp, traj_xyz, '-', "black", "ground truth")
    # ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    plt.pause(0.1)
    plt.waitforbuttonpress(100)
    ax.clear()


def plot_traj3d(ax, stamps, traj, style, color, label):
    stamps.sort()
    interval = np.median([s - t for s, t in zip(stamps[1:], stamps[:-1])]) * 3
    start = 0
    last = 0
    for i, (t2, t1) in enumerate(zip(stamps[1:], stamps[:-1])):
        last = i+1
        if t2 - t1 > interval:
            if last - start > 10:
                ax.plot(traj[start:last, 0], traj[start:last, 1], traj[start:last, 2],
                        linestyle=style, color=color)
            start = last

    if last - start > 10:
        ax.plot(traj[start:last, 0], traj[start:last, 1], traj[start:last, 2],
                linestyle=style, color=color)


def main():
    np.set_printoptions(precision=5, suppress=True)
    draw_trajectory("tum_vi")
    draw_trajectory("euroc_mav")


if __name__ == "__main__":
    main()
