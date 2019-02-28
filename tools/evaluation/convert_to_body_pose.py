import os
import os.path as op
import glob
import numpy as np
import yaml

import settings
from define_paths import *
import evaluation.rotation as rotation


def read_euroc_pose_cam_body(dataset_path):
    seq_paths = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/") if
                 op.isdir(s + "mav0")]
    camf = open(op.join(seq_paths[0], "mav0/cam0/sensor.yaml"))
    cam_param = yaml.load(camf)
    cam2body = np.array(cam_param['T_BS']['data']).reshape((4, 4))
    print("euroc camera to body {}\n{}".format(np.linalg.det(cam2body[:3, :3]), cam2body))
    return cam2body


def read_tumvi_pose_cam_body(dataset_path):
    seq_paths = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/") if
                 op.isdir(s + "mav0")]
    camf = open(op.join(seq_paths[0], "dso/camchain.yaml"))
    cam_param = yaml.load(camf)
    cam2body = np.array(cam_param['cam0']['T_cam_imu']).reshape((4, 4))
    # tumvi's extrinsic means IMU frame w.t.t camera
    # but we need camera frame w.r.t IMU
    cam2body = np.linalg.inv(cam2body)
    # offset = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # cam2body = np.dot(offset, cam2body)
    print("tumvi camera to body {}\n{}".format(np.linalg.det(cam2body[:3, :3]), cam2body))
    return cam2body


def convert_to_body_pose(data_root, dataset, alg_prefix, cam2body=None, alg_offset=None):
    srcfiles = glob.glob(op.join(data_root, dataset, alg_prefix + "*"))
    assert len(srcfiles) > 0, op.join(data_root, dataset, alg_prefix + "*")
    tobody_pose = np.identity(4)
    if alg_offset is not None:
        tobody_pose = np.dot(tobody_pose, alg_offset)
    if cam2body is not None:
        tobody_pose = np.dot(tobody_pose, cam2body)
    print("\n===== dataset:", dataset, ", algorithm:", alg_prefix, "\noffset to body pose\n", tobody_pose)

    for srcfile in srcfiles:
        poses = np.loadtxt(srcfile)
        dstfile = srcfile.replace("/pose/", "/pose_body/")

        new_poses = convert_pose(poses, tobody_pose)

        print("save to", dstfile)
        os.makedirs(op.dirname(dstfile), exist_ok=True)
        np.savetxt(dstfile, new_poses, fmt="%1.6f")


def convert_pose(seq_poses, tobody_pose_mat):
    print("start transforming ...", seq_poses.shape)
    nonzero_seq_poses = remove_zero_frames(seq_poses)

    new_seq_poses = []
    first_pose_mat = None
    for i, pose_vec in enumerate(nonzero_seq_poses):
        timestamp = pose_vec[0]
        frame_time = pose_vec[-1] if pose_vec.size == 9 else -1
        algo_pose_mat = rotation.transform44(pose_vec)

        # remove offset pose to get body (IMU) pose
        # T_body = T_cam * inv(T_body_cam)
        body_pose_mat = np.dot(algo_pose_mat, np.linalg.inv(tobody_pose_mat))

        # divide by first pose for trajectory starts from the zero pose
        if i == 0:
            first_pose_mat = body_pose_mat
        zerobase_body_mat = np.dot(np.linalg.inv(first_pose_mat), body_pose_mat)
        zerobase_body_vec = rotation.pose_quat(zerobase_body_mat, timestamp)

        if frame_time:
            zerobase_body_vec = np.append(zerobase_body_vec, frame_time)
        new_seq_poses.append(zerobase_body_vec)

    new_seq_poses = np.array(new_seq_poses)
    print("transformed to camera poses before and after: {} \n{} \n{}".
          format(new_seq_poses.shape, np.array(nonzero_seq_poses)[:3, 1:8], new_seq_poses[:3, 1:8]))
    return new_seq_poses


def remove_zero_frames(trajectory):
    # remove zero poses
    nonzero_traj = []
    for pose in trajectory:
        posit_norm = np.linalg.norm(pose[1:4])
        if posit_norm > 1.0E-9:
            nonzero_traj.append(pose)
        else:
            print("remove zero pose")

    # remove stopping poses
    moving_traj = [nonzero_traj[0]]
    for curpose, prvpose in zip(nonzero_traj[1:], nonzero_traj[:-1]):
        move = np.linalg.norm(curpose[1:4] - prvpose[1:4])
        if move > 1.0E-9:
            moving_traj.append(curpose)
        else:
            print("remove stopping pose: \n{} \n{}".format(curpose[1:], prvpose[1:]))

    return moving_traj


def main():
    np.set_printoptions(precision=5, suppress=True)
    euroc_c2b = read_euroc_pose_cam_body(EUROC_PATH)
    tumvi_c2b = read_tumvi_pose_cam_body(TUMVI_PATH)

    for dataset, cam2body in zip(["euroc_mav", "tum_vi"], [euroc_c2b, tumvi_c2b]):
        convert_to_body_pose(op.join(OUTPUT_PATH, "pose"), dataset, "orb2", cam2body)
        convert_to_body_pose(op.join(OUTPUT_PATH, "pose"), dataset, "svo2", cam2body)
        convert_to_body_pose(op.join(OUTPUT_PATH, "pose"), dataset, "vinsfs")
        convert_to_body_pose(op.join(OUTPUT_PATH, "pose"), dataset, "rovioli")


if __name__ == "__main__":
    main()
