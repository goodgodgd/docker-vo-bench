import numpy as np
import cv2
import sys
import yaml


def tum_vi():
    sys.path.append("/usr/local/lib/python3.5/dist-packages/cv2")
    cam0_params = [0.373004838186, 0.372994740336, 0.498890050897, 0.502729380663, 0.00348238940225, 0.000715034845216, -0.00205323614187, 0.000202936735918]
    cam1_params = [0.371957753309, 0.371942262641, 0.494334955407, 0.498861778606, 0.00340031707904, 0.00176627815347, -0.00266312569782, 0.000329951742393]
    img_size = [512, 512]
    cam0_mat, cam0_dist, cam0_opt_mat = get_opt_new_cam_mat(cam0_params, img_size)
    cam1_mat, cam1_dist, cam1_opt_mat = get_opt_new_cam_mat(cam1_params, img_size)
    print("cam0 opt matrix\n", cam0_opt_mat)
    print("cam1 opt matrix\n", cam1_opt_mat)

    srcyaml = "/work/ORB_SLAM2/Examples/Stereo/EuRoC.yaml"
    dstyaml = "/work/ORB_SLAM2/Examples/Stereo/TUM_VI.yaml"
    yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)
    yaml.add_representer(np.ndarray, opencv_matrix_representer)

    baseline = 0.10106110275180535

    with open(srcyaml, 'r') as fin:
        data = yaml.load(fin)
        data["Camera.fx"] = float(cam0_opt_mat[0, 0])
        data["Camera.fy"] = float(cam0_opt_mat[1, 1])
        data["Camera.cx"] = float(cam0_opt_mat[0, 2])
        data["Camera.cy"] = float(cam0_opt_mat[1, 2])
        data["Camera.width"] = img_size[0]
        data["Camera.height"] = img_size[1]
        data["Camera.bf"] = float(cam0_opt_mat[0, 0] * baseline)

        data["LEFT.width"] = img_size[0]
        data["LEFT.height"] = img_size[1]
        print("shape", cam0_dist.shape, cam1_dist.shape, cam1_mat.shape)
        data["LEFT.D"] = cam0_dist
        data["LEFT.K"] = cam0_mat
        data["LEFT.R"] = np.identity(3)
        data["LEFT.P"] = cam0_opt_mat
        
        data["RIGHT.width"] = img_size[0]
        data["RIGHT.height"] = img_size[1]
        data["RIGHT.D"] = cam1_dist
        data["RIGHT.K"] = cam1_mat
        data["RIGHT.R"] = np.identity(3)
        data["RIGHT.P"] = cam1_opt_mat
        data["Fisheye"] = 1

        with open(dstyaml, 'w') as fout:
            yaml.dump(data, fout)


def get_opt_new_cam_mat(params, img_size):
    cam_mat, cam_dist = parse_params_dso(params, img_size)
    print("cam raw matrix\n", cam_mat)
    cam_opt_mat, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=cam_mat, distCoeffs=cam_dist,
                                                     imageSize=tuple(img_size), alpha=0)
    # cam_opt_mat = np.concatenate([cam_opt_mat, np.zeros((3, 1))], axis=1)
    return cam_mat, cam_dist, cam_opt_mat


# params: fx fy cx cy k1 k2 r1 r2
def parse_params_dso(params, img_size):
    cam_mat = np.array([[params[0], 0, params[2]],
                        [0, params[1], params[3]],
                        [0, 0, 1]])
    cam_mat[0] = cam_mat[0] * img_size[0]
    cam_mat[1] = cam_mat[1] * img_size[1]
    dist = params[4:]
    # dist.append(0)
    cam_dist = np.array([dist])
    return cam_mat, cam_dist


# A yaml constructor is for loading from a yaml node.
# This is taken from @misha 's answer: http://stackoverflow.com/a/15942429
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat


# A yaml representer is for dumping structs into a yaml node.
# So for an opencv_matrix type (to be compatible with c++'s FileStorage)
# we save the rows, cols, type and flattened-data
def opencv_matrix_representer(dumper, mat):
    if len(mat.shape) > 1:
        cols = int(mat.shape[1])
    else:
        cols = 1
    mapping = {'rows': int(mat.shape[0]), 'cols': cols, 'dt': 'd', 'data': mat.reshape(-1).tolist()}
    return dumper.represent_mapping(u"tag:yaml.org,2002:opencv-matrix", mapping)


if __name__ == "__main__":
    tum_vi()
