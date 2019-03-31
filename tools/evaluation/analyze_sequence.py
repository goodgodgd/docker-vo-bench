import os
import os.path as op
import numpy as np
import pandas as pd
import settings
from define_paths import *
import evaluation.eval_common as ec
import evaluation.rotation as rotation


def analyze_sequences(dataset):
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    sequences = ec.list_sequences(gtruth_path)
    columns = ["name", "total_time", "gt_time", "max tran", "mean tran", "max rota", "mean rota",
               "size_x", "size_y", "size_z"]
    seq_info = []
    result_path = op.join(OUTPUT_PATH, "eval_result", "seq_info")

    for seq_name in sequences:
        print("sequence name:", seq_name)
        gtruth_file = op.join(gtruth_path, seq_name + ".csv")
        trajectory = pd.read_csv(gtruth_file)
        trajectory = trajectory.values
        timestamp = trajectory[:, 0]
        traj_xyz = trajectory[:, 1:4]
        traj_quat = trajectory[:, 4:8]
        extent = np.max(traj_xyz, axis=0) - np.min(traj_xyz, axis=0)

        mean_time = 0.5
        frames = frames_per_time(timestamp, mean_time)
        trn_vel = translation_velocity(timestamp, traj_xyz, mean_time, frames)
        rot_vel = rotation_velocity(timestamp, traj_quat, mean_time, frames)

        max_tvel = np.max(trn_vel)
        med_tvel = np.mean(trn_vel)
        max_rvel = np.max(rot_vel)
        med_rvel = np.mean(rot_vel)
        total_time = timestamp[-1] - timestamp[0]
        gt_time = ec.accumulate_connected_time(timestamp, max_diff=0.5)

        info = [seq_name, total_time, gt_time, max_tvel, med_tvel, max_rvel, med_rvel,
                extent[0], extent[1], extent[2]]
        seq_info.append(info)

    seq_info = pd.DataFrame(data=seq_info, columns=columns)
    print(seq_info)
    filename = op.join(result_path, dataset + ".csv")
    if not op.isdir(result_path):
        os.makedirs(result_path)
    seq_info.to_csv(filename, encoding="utf-8", float_format="%1.5f")


def frames_per_time(timestamp, duration):
    timediffs = timestamp[1:] - timestamp[:-1]
    timediffs = timediffs[timediffs < 0.1]
    mean_frame_time = np.mean(timediffs)
    return int(duration // mean_frame_time)


def translation_velocity(timestamp, traj_xyz, mean_time, frames: int):
    movements = traj_xyz[frames:, :] - traj_xyz[:-frames, :]
    movements = np.sqrt(np.sum(movements * movements, axis=1))
    timediffs = timestamp[frames:] - timestamp[:-frames]
    mask = (timediffs < mean_time * 2)
    velocity = movements[mask] / timediffs[mask]
    return velocity
    
    
def rotation_velocity(timestamp, traj_quat, mean_time, frames: int):
    rotangles = []
    for i in range(len(traj_quat) - frames):
        R1_mat = rotation.rotation33(traj_quat[i  , :])
        R2_mat = rotation.rotation33(traj_quat[i+frames, :])
        rel_rot = np.linalg.inv(R1_mat) * R2_mat
        angle, axis = rotation.angle_axis_from_mat(rel_rot)
        rotangles.append(angle)

    rotangles = np.array(rotangles)
    timediffs = timestamp[frames:] - timestamp[:-frames]
    mask = (timediffs < mean_time * 2)
    rot_velocity = rotangles[mask] / timediffs[mask]
    return rot_velocity


def main():
    np.set_printoptions(precision=5, suppress=True)
    analyze_sequences("tum_vi")
    analyze_sequences("euroc_mav")


if __name__ == "__main__":
    main()
