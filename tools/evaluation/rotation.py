import numpy as np
from pyquaternion import Quaternion

_EPS = np.finfo(float).eps * 4.0


# Source: TUM RGB-D Dataset tools
def transform44(l):
    """
    Generate a 4x4 homogeneous transformation matrix from a 3D point and unit quaternion.

    Input:
    l -- tuple consisting of (stamp,tx,ty,tz,qx,qy,qz,qw) where
         (tx,ty,tz) is the 3D position and (qx,qy,qz,qw) is the unit quaternion.

    Output:
    matrix -- 4x4 homogeneous transformation matrix
    """
    t = l[1:4]
    q = np.array(l[4:8], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    assert (nq-1) < 1.0E-3, "non unit quaternion: {} {}".format(q, nq)
    if nq < _EPS:
        print("eps", _EPS, nq)
        matrix = np.identity(4)
        matrix[:3, 3] = np.array(t, dtype=np.float64)
        return matrix

    q *= np.sqrt(1.0 / nq)
    q = np.outer(q, q) * 2.0
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], t[0]),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], t[1]),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], t[2]),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)


def pose_quat(T, timestamp):
    quat = Quaternion(matrix=T[:3, :3])
    assert np.allclose(quat.rotation_matrix, T[:3, :3])
    quat = quat.unit
    quat = quat.elements
    quat = np.array([quat[1], quat[2], quat[3], quat[0]])
    if quat[3] < 0:
        quat *= -1.0
    assert np.abs(np.linalg.norm(quat) - 1.) < 0.001, \
        "Non unit quaternion {}".format(quat, np.linalg.norm(quat))
    tran = T[:3, 3].T
    pose = np.append(tran, quat)
    pose = np.insert(pose, 0, timestamp)
    assert len(pose) == 8
    return pose


def normalize_so3(rot):
    # normalize rotation matrix
    U, S, Vh = np.linalg.svd(rot)
    S = np.diag(S)
    assert np.allclose(rot, np.dot(U, np.dot(S, Vh))), "svd error\n{}\n{}".format(rot, np.dot(U, np.dot(S, Vh)))
    S = np.identity(3)
    return np.dot(U, np.dot(S, Vh))

