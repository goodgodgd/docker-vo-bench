#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import os
import sys
import numpy as np
import argparse
import associate
import matplotlib.backends.backend_qt5agg


def evaluate_ate(first_timetraj, second_timetraj, offset=0.0, max_difference=0.02,
                 save_associations=None, plot=None, major_axes=None, plot_3d=None, verbose=False):
    """
    Copmute ATE
    Input:
    first_list -- {time stamp: pose}
    second_list -- {time stamp: pose}
    """

    matches = associate.associate(first_timetraj, second_timetraj, offset, max_difference)
    if len(matches) < 2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and "
                 "estimated trajectory! Did you choose the correct sequence?")

    # txyz: 4xn matrix = rows of timestamp, x, y, z
    first_txyz, first_xyz_match = extract_trajmatrix(first_timetraj, [a for a, b in matches], major_axes)
    second_txyz, second_xyz_match = extract_trajmatrix(second_timetraj, [b for a, b in matches], major_axes)
    print("data shapes", first_txyz.shape, first_xyz_match.shape, second_txyz.shape, second_xyz_match.shape)

    rot, trans, trans_error = align(second_xyz_match, first_xyz_match)
    second_txyz[1:, :] = rot * second_txyz[1:, :] + trans
    second_xyz_match = rot * second_xyz_match + trans

    association = np.array([[a, x1, y1, z1, b, x2, y2, z2]
                            for (a, b), (x1, y1, z1), (x2, y2, z2) in
                            zip(matches, first_xyz_match.transpose().A,
                                second_xyz_match.transpose().A)])

    print_stats(trans_error, verbose)

    if save_associations:
        print("save associations:", save_associations)
        np.savetxt(save_associations, association, fmt="%1.5f")

    if plot:
        plot2d(first_txyz, first_xyz_match, second_txyz, second_xyz_match, plot)

    if plot_3d:
        plot3d(first_txyz, first_xyz_match, second_txyz, second_xyz_match, plot)

    association = np.array(association, dtype=np.float64)
    return rot, trans, trans_error, association


def extract_trajmatrix(traj, match_times, major_axes):
    timestamp = list(traj.keys())
    timestamp.sort()
    xyz = np.array([traj[t][:3] for t in timestamp]).transpose()
    txyz = np.vstack([timestamp, xyz])
    txyz = np.matrix(txyz.astype(float))
    xyz_match = np.array([traj[t][:3] for t in match_times]).transpose()
    xyz_match = np.matrix(xyz_match.astype(float))

    if major_axes == "yz":
        txyz = np.vstack([txyz[0, :], txyz[2, :], txyz[3, :], txyz[1, :]])
        xyz_match = np.vstack([xyz_match[1, :], xyz_match[2, :], xyz_match[0, :]])
    return txyz, xyz_match


def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    
    """

    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
    U, d, Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity(3))

    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[2, 2] = -1

    rot = U*S*Vh
    trans = model.mean(1) - rot * data.mean(1)
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error), 0)).A[0]

    return rot, trans, trans_error


def plot2d(first_txyz, first_xyz_match, second_txyz, second_xyz_match, save_name):
    """
    Plot a trajectory using matplotlib.

    Input:
    first_stamps -- timestamps of first_xyz_full
    first_xyz -- first traj at matches
    first_xyz_full -- full traj at first_stamps
    second_stamps -- timestamps of second_xyz_full
    second_xyz_aligned -- second traj at matches
    second_xyz_full_aligned -- full traj at second_stamps
    save_name -- figure file name to save

    """
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111)
    algorithm, sequence = parse_name(save_name)
    plot_traj(ax, first_txyz, False, '-', "black", "ground truth")
    plot_traj(ax, second_txyz, False, '-', "blue", algorithm)

    label = "difference"
    first_match = first_xyz_match[:, 0:-1:3]
    second_match = second_xyz_match[:, 0:-1:3]
    for (x1, y1, z1), (x2, y2, z2) in zip(first_match.transpose().A, second_match.transpose().A):
        ax.plot([x1, x2], [y1, y2], '-', color="red", label=label)
        label = ""

    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.title(sequence)
    plt.savefig(save_name, dpi=90)


def plot3d(first_txyz, first_xyz_match, second_txyz, second_xyz_match, save_name):
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import axes3d, Axes3D

    fig = plt.figure()
    ax = Axes3D(fig)

    algorithm, sequence = parse_name(save_name)
    plot_traj(ax, first_txyz, True, '-', "black", "ground truth")
    plot_traj(ax, second_txyz, True, '-', "blue", algorithm)

    label = "difference"
    first_match = first_xyz_match[:, 0:-1:3]
    second_match = second_xyz_match[:, 0:-1:3]
    for (x1, y1, z1), (x2, y2, z2) in zip(first_match.transpose().A, second_match.transpose().A):
        ax.plot([x1, x2], [y1, y2], [z1, z2], '-', color="red", label=label)
        label = ""

    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    plt.title(sequence)
    plt.show(block=True)
    # plt.savefig(save_name, dpi=90)


def parse_name(name):
    filename = os.path.basename(name)
    splits = filename.split("_")
    algorithm = "_".join(splits[1:-2])
    sequence = splits[-2]
    return algorithm, sequence


def plot_traj(ax, txyz, is3d, style, color, label):
    txyz = np.array(txyz)
    timestamp = txyz[0, :]
    x = txyz[1, :]
    y = txyz[2, :]
    z = txyz[3, :]

    interval = np.median([s - t for s, t in zip(timestamp[1:], timestamp[:-1])]) * 3
    start = 0
    last = 0
    for i, (t2, t1) in enumerate(zip(timestamp[1:], timestamp[:-1])):
        last = i+1
        if t2 - t1 > interval:
            if last - start > 10:
                if is3d:
                    ax.plot(x[start:last], y[start:last], z[start:last],
                            linestyle=style, color=color, label=label)
                else:
                    ax.plot(x[start:last], y[start:last], linestyle=style, color=color, label=label)
                label = ""
            start = last

    if is3d:
        ax.plot(x[start:last], y[start:last], z[start:last],
                linestyle=style, color=color, label=label)
    else:
        ax.plot(x[start:last], y[start:last], linestyle=style, color=color, label=label)


def print_stats(trans_error, verbose):
    if verbose:
        print("compared_pose_pairs %d pairs" % (len(trans_error)))
        print("absolute_translational_error.rmse %f m" % np.sqrt(
            np.dot(trans_error, trans_error) / len(trans_error)))
        print("absolute_translational_error.mean %f m" % np.mean(trans_error))
        print("absolute_translational_error.median %f m" % np.median(trans_error))
        print("absolute_translational_error.std %f m" % np.std(trans_error))
        print("absolute_translational_error.min %f m" % np.min(trans_error))
        print("absolute_translational_error.max %f m" % np.max(trans_error))
    else:
        print("absolute_translational_error.rmse: %f" %
              np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)))


def main():
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', default=0.0, help='time offset added to the timestamps of the second file (default: 0.0)')
    parser.add_argument('--scale', default=1.0, help='scaling factor for the second trajectory (default: 1.0)')
    parser.add_argument('--max_difference', default=0.02, help='maximally allowed time difference for matching entries (default: 0.02)')
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--plot3D', help='plot the first and the aligned second trajectory to as interactive 3D plot (format: png)', action = 'store_true')
    parser.add_argument('--verbose', help='print(all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file)
    second_list = associate.read_file_list(args.second_file)
    evaluate_ate(first_list, second_list, args.offset, args.scale,
                 args.max_difference, args.save, args.save_associations,
                 args.plot, args.plot3D, args.verbose)


if __name__ == "__main__":
    main()
