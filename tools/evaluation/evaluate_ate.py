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

import sys
import numpy
import argparse
import associate
import matplotlib.backends.backend_qt5agg


def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    
    """
    numpy.set_printoptions(precision=5,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = data.mean(1) - rot * model.mean(1)
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error


def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)


def plot_traj3D(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    z = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
            z.append(traj[i][2])
        elif len(x)>0:
            ax.plot(x,y,z,style,color=color,label=label)
            
            label=""
            x=[]
            y=[]
            z=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,z,style,color=color,label=label)          


def plot2d(first_stamps, first_xyz, first_xyz_full,
           second_stamps, second_xyz_aligned, second_xyz_full_aligned,
           matches, save_name):
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plot_traj(ax, first_stamps, first_xyz_full.transpose().A, '-', "black",
              "ground truth")
    plot_traj(ax, second_stamps, second_xyz_full_aligned.transpose().A, '-', "blue",
              "estimated")

    label = "difference"
    for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(matches, first_xyz.transpose().A,
                                                  second_xyz_aligned.transpose().A):
        ax.plot([x1, x2], [y1, y2], '-', color="red", label=label)
        label = ""

    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.savefig(save_name, dpi=90)


def plot3d(first_stamps, first_xyz, first_xyz_full,
           second_stamps, second_xyz_aligned, second_xyz_full_aligned,
           matches, save_name):
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import axes3d, Axes3D

    fig = plt.figure()
    ax = Axes3D(fig)
    #        ax = fig.add_subplot(111)
    plot_traj3D(ax, first_stamps, first_xyz_full.transpose().A, '-', "black",
                "ground truth")
    plot_traj3D(ax, second_stamps, second_xyz_full_aligned.transpose().A, '-', "blue",
                "estimated")

    label = "difference"
    for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(matches, first_xyz.transpose().A,
                                                  second_xyz_aligned.transpose().A):
        ax.plot([x1, x2], [y1, y2], [z1, z2], '-', color="red", label=label)
        label = ""
    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    print("Showing")
    plt.show(block=True)
    plt.savefig(save_name, dpi=90)


def remove_zero_frames(traj_est):
    timestamps = list(traj_est.keys())
    timestamps.sort()

    # remove default poses
    for time in timestamps:
        pose = numpy.array(traj_est[time], dtype=numpy.float64)
        posit_sq = numpy.linalg.norm(pose[:3])
        if posit_sq < 1.0E-9:
            del traj_est[time]

    # remove stopping poses
    for time, time_bef in zip(timestamps[1:], timestamps[:-1]):
        if time in traj_est.keys() and time_bef in traj_est.keys():
            pose = numpy.array(traj_est[time], dtype=numpy.float64)
            pose_bef = numpy.array(traj_est[time_bef], dtype=numpy.float64)
            posit_diff = (pose[:3] - pose_bef[:3]).T
            move = numpy.linalg.norm(posit_diff)
            if move < 1.0E-9:
                del traj_est[time]

    return traj_est


def evaluate_ate(first_file, second_file, offset=0.0, scale=1.0, max_difference=0.02,
                 save=None, save_associations=None, plot=None, plot3D=None, verbose=False):

    first_list = associate.read_file_list(first_file)
    second_list = associate.read_file_list(second_file)

    second_list = remove_zero_frames(second_list)

    matches = associate.associate(first_list, second_list, float(offset),
                                  float(max_difference))
    print("[ATE] gt len, esti len, matches", len(first_list.keys()), len(second_list.keys()), len(matches))
    print("est poses", second_list[1403636624.763556], "\n", second_list[1403636634.763556])

    if len(matches) < 2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and "
                 "estimated trajectory! Did you choose the correct sequence?")

    first_xyz = numpy.matrix(
        [[float(value) for value in first_list[a][0:3]] for a, b in matches]).transpose()
    second_xyz = numpy.matrix(
        [[float(value) * float(scale) for value in second_list[b][0:3]] for a, b in
         matches]).transpose()
    rot, trans, trans_error = align(second_xyz, first_xyz)

    second_xyz_aligned = rot * second_xyz + trans

    first_stamps = list(first_list.keys())
    first_stamps.sort()
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in
                                   first_stamps]).transpose()

    second_stamps = list(second_list.keys())
    second_stamps.sort()
    second_xyz_full = numpy.matrix(
        [[float(value) * float(scale) for value in second_list[b][0:3]] for b in
         second_stamps]).transpose()
    second_xyz_full_aligned = rot * second_xyz_full + trans

    association = ["%f %f %f %f %f %f %f %f" % (a, x1, y1, z1, b, x2, y2, z2)
                   for (a, b), (x1, y1, z1), (x2, y2, z2) in
                   zip(matches, first_xyz.transpose().A,
                       second_xyz_aligned.transpose().A)]
    gt_tstamps = numpy.array(list(first_list.keys()))

    if verbose:
        print("compared_pose_pairs %d pairs" % (len(trans_error)))

        print("absolute_translational_error.rmse %f m" % numpy.sqrt(
            numpy.dot(trans_error, trans_error) / len(trans_error)))
        print("absolute_translational_error.mean %f m" % numpy.mean(trans_error))
        print("absolute_translational_error.median %f m" % numpy.median(trans_error))
        print("absolute_translational_error.std %f m" % numpy.std(trans_error))
        print("absolute_translational_error.min %f m" % numpy.min(trans_error))
        print("absolute_translational_error.max %f m" % numpy.max(trans_error))
    else:
        print("%f" % numpy.sqrt(numpy.dot(trans_error, trans_error) / len(trans_error)))

    if save_associations:
        file = open(save_associations, "w")
        file.write("\n".join(association))
        file.close()

    if save:
        file = open(save, "w")
        file.write("\n".join(
            ["%f " % stamp + " ".join(["%f" % d for d in line]) for stamp, line in
             zip(second_stamps, second_xyz_full_aligned.transpose().A)]))
        file.close()

    if plot:
        plot2d(first_stamps, first_xyz, first_xyz_full,
               second_stamps, second_xyz_aligned, second_xyz_full_aligned,
               matches, plot)

    if plot3D:
        plot3d(first_stamps, first_xyz, first_xyz_full,
               second_stamps, second_xyz_aligned, second_xyz_full_aligned,
               matches, plot3D)

    return rot, trans, trans_error, association, gt_tstamps


if __name__ == "__main__":
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
    evaluate_ate(args.first_file, args.second_file, args.offset, args.scale, 
                 args.max_difference, args.save, args.save_associations,
                 args.plot, args.plot3D, args.verbose)
