import os
import os.path as op
import subprocess
import argparse
import glob

ORB2_ROOT = "/work/ORB_SLAM2"
VOCABULARY = ORB2_ROOT + "/Vocabulary/ORBvoc.txt"
DATA_ROOT = "/data"
OUTPUT_ROOT = "/work/output"
TEST_NUM = 5


def run_orb2(opt):
    check_base_paths()
    print("exef", opt.exec)
    if opt.exec == "all":
        command_makers = [stereo_euroc, stereo_kitti, mono_euroc, mono_kitti, mono_tum_rgbd]
        commands = []
        configs = []
        for lc in [0, 1]:
            opt.loopclosing = lc
            for tid in range(TEST_NUM):
                opt.test_id = tid
                for cmdmaker in command_makers:
                    cmds, cfgs = cmdmaker(opt)
                    commands.extend(cmds)
                    configs.extend(cfgs)
    elif opt.exec == "mono_tum_rgbd":
        commands, configs = mono_tum_rgbd(opt)
    elif opt.exec == "mono_kitti":
        commands, configs = mono_kitti(opt)
    elif opt.exec == "mono_euroc":
        commands, configs = mono_euroc(opt)
    elif opt.exec == "stereo_kitti":
        commands, configs = stereo_kitti(opt)
    elif opt.exec == "stereo_euroc":
        commands, configs = stereo_euroc(opt)
    else:
        raise FileNotFoundError()

    for cmd, cfg in zip(commands, configs):
        outfile = cmd[-1]
        os.makedirs(op.dirname(outfile), exist_ok=True)
        print("\n===== RUN ORB_SLAM2\nconfig: {}\ncmd: {}\n".format(cfg, cmd))
        subprocess.run(cmd)
        subprocess.run(["chmod", "-R", "a+rw", OUTPUT_ROOT])
        assert op.isfile(outfile), "===== ERROR: output file was NOT created: {}".format(outfile)


def check_base_paths():
    assert op.isdir(ORB2_ROOT), "ORB SLAM2 dir doesn't exist"
    assert op.isfile(VOCABULARY), "Vocabulary file doesn't exist"
    assert op.isdir(DATA_ROOT), "dataset dir doesn't exist"
    assert op.isdir(OUTPUT_ROOT), "output dir doesn't exist"


# ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml
# PATH_TO_SEQUENCE_FOLDER loop_closing_on output_file
def mono_tum_rgbd(opt):
    exec_path = op.join(ORB2_ROOT, "Examples/Monocular")
    executer = op.join(exec_path, "mono_tum")
    dataset_path = op.join(DATA_ROOT, "tum_rgbd")
    sequences = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/")]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    commands = []
    configs = []
    for si, seq_path in enumerate(sequences):
        if "freiburg1" in seq_path:
            config_file = op.join(exec_path, "TUM1.yaml")
        elif "freiburg2" in seq_path:
            config_file = op.join(exec_path, "TUM2.yaml")
        elif "freiburg3" in seq_path:
            config_file = op.join(exec_path, "TUM3.yaml")
        else:
            raise FileNotFoundError()

        output_dir = "orb2_vo_mono" if opt.loopclosing == 0 else "orb2_slam_mono"
        filename = op.basename(seq_path).replace("rgbd_dataset_freiburg", "tum_fr") \
                   + "_t{}.txt".format(opt.test_id)
        output_file = op.join(OUTPUT_ROOT, output_dir, "pose", filename)
        cmd = [executer, VOCABULARY, config_file, seq_path, str(opt.loopclosing), output_file]
        print("===== mono tum command =====\n", " ".join(cmd))
        commands.append(cmd)
        conf = {"executer": "mono_tum", "loop closing": opt.loopclosing,
                "test id": opt.test_id, "sequence": si}
        configs.append(conf)
    return commands, configs


# ./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml
# PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBE loop_closing_on output_file
def mono_kitti(opt):
    exec_path = op.join(ORB2_ROOT, "Examples/Monocular")
    executer = op.join(exec_path, "mono_kitti")
    dataset_path = op.join(DATA_ROOT, "kitti_odom", "sequences")
    sequences = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/")]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    commands = []
    configs = []
    for si, seq_path in enumerate(sequences):
        seq_id = int(op.basename(seq_path))
        if seq_id < 3:
            config_file = op.join(exec_path, "KITTI00-02.yaml")
        elif seq_id == 3:
            config_file = op.join(exec_path, "KITTI03.yaml")
        elif seq_id > 3:
            config_file = op.join(exec_path, "KITTI04-12.yaml")
        else:
            raise FileNotFoundError()

        output_dir = "orb2_vo_mono" if opt.loopclosing == 0 else "orb2_slam_mono"
        output_file = op.join(OUTPUT_ROOT, output_dir, "pose", "kitti_odom_"
                              + op.basename(seq_path) + "_t{}.txt".format(opt.test_id))
        cmd = [executer, VOCABULARY, config_file, seq_path, str(opt.loopclosing), output_file]
        print("===== mono kitti command\n", " ".join(cmd))
        commands.append(cmd)
        conf = {"executer": "mono_tum", "loop closing": opt.loopclosing,
                "test id": opt.test_id, "sequence": si}
        configs.append(conf)
    return commands, configs


# ./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml
# PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data
# Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt loop_closing_on output_file
def mono_euroc(opt):
    exec_path = op.join(ORB2_ROOT, "Examples/Monocular")
    executer = op.join(exec_path, "mono_euroc")
    dataset_path = op.join(DATA_ROOT, "euroc")
    sequences = [s + "mav0/cam0/data" for s in glob.glob(dataset_path + "/*/")]
    sequences = [seq.replace("mav0/cam0", "cam0") if "MH_" in seq else seq
                 for seq in sequences]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    config_file = op.join(exec_path, "EuRoC.yaml")
    commands = []
    configs = []
    for si, seq_path in enumerate(sequences):
        timefile = seq_path.split("/")[-4]
        timefile = timefile.split("_")
        timefile = op.join(exec_path, "EuRoC_TimeStamps", timefile[0] + timefile[1] + ".txt")

        output_dir = "orb2_vo_mono" if opt.loopclosing == 0 else "orb2_slam_mono"
        filename = "euroc_" + op.basename(timefile).replace(".txt", "_t{}.txt".format(opt.test_id))
        output_file = op.join(OUTPUT_ROOT, output_dir, "pose", filename)
        cmd = [executer, VOCABULARY, config_file, seq_path, timefile, str(opt.loopclosing), output_file]
        print("===== mono euroc command\n", " ".join(cmd))
        commands.append(cmd)
        conf = {"executer": "mono_tum", "loop closing": opt.loopclosing,
                "test id": opt.test_id, "sequence": si}
        configs.append(conf)
    return commands, configs


# ./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml
# PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER loop_closing_on output_file
def stereo_kitti(opt):
    exec_path = op.join(ORB2_ROOT, "Examples/Stereo")
    executer = op.join(exec_path, "stereo_kitti")
    dataset_path = op.join(DATA_ROOT, "kitti_odom", "sequences")
    sequences = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/")]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    commands = []
    configs = []
    for si, seq_path in enumerate(sequences):
        seq_id = int(op.basename(seq_path))
        if seq_id < 3:
            config_file = op.join(exec_path, "KITTI00-02.yaml")
        elif seq_id == 3:
            config_file = op.join(exec_path, "KITTI03.yaml")
        elif seq_id > 3:
            config_file = op.join(exec_path, "KITTI04-12.yaml")
        else:
            raise FileNotFoundError()

        output_dir = "orb2_vo_stereo" if opt.loopclosing == 0 else "orb2_slam_stereo"
        output_file = op.join(OUTPUT_ROOT, output_dir, "pose", "kitti_odom_"
                              + op.basename(seq_path) + "_t{}.txt".format(opt.test_id))
        cmd = [executer, VOCABULARY, config_file, seq_path, str(opt.loopclosing), output_file]
        print("===== stereo kitti command\n", " ".join(cmd))
        commands.append(cmd)
        conf = {"executer": "mono_tum", "loop closing": opt.loopclosing,
                "test id": opt.test_id, "sequence": si}
        configs.append(conf)
    return commands, configs


# ./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml
# PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data
# Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt loop_closing_on output_file
def stereo_euroc(opt):
    exec_path = op.join(ORB2_ROOT, "Examples/Stereo")
    executer = op.join(exec_path, "stereo_euroc")
    dataset_path = op.join(DATA_ROOT, "euroc")
    sequences = [s + "mav0/cam0/data" for s in glob.glob(dataset_path + "/*/")]
    sequences = [seq.replace("mav0/cam0", "cam0") if "MH_" in seq else seq
                 for seq in sequences]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    config_file = op.join(exec_path, "EuRoC.yaml")
    commands = []
    configs = []
    for si, right_seq in enumerate(sequences):
        left_seq = right_seq.replace("cam0", "cam1")
        timefile = right_seq.split("/")[-4]
        timefile = timefile.split("_")
        timefile = op.join(exec_path, "EuRoC_TimeStamps", timefile[0] + timefile[1] + ".txt")

        output_dir = "orb2_vo_stereo" if opt.loopclosing == 0 else "orb2_slam_stereo"
        filename = "euroc_" + op.basename(timefile).replace(".txt", "_t{}.txt".format(opt.test_id))
        output_file = op.join(OUTPUT_ROOT, output_dir, "pose", filename)
        cmd = [executer, VOCABULARY, config_file, right_seq, left_seq, timefile,
               str(opt.loopclosing), output_file]
        print("===== stereo euroc command\n", " ".join(cmd))
        commands.append(cmd)
        conf = {"executer": "mono_tum", "loop closing": opt.loopclosing,
                "test id": opt.test_id, "sequence": si}
        configs.append(conf)
    return commands, configs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("exec", type=str, help="orbslam2 executer file, "
                        "options: all, mono_tum, mono_kitti, mono_euroc, rgbd_tum"
                        ", stereo_kitti, stereo_euroc")
    parser.add_argument("-s", "--seq_idx", default=-1, type=int,
                        help="int: index of sequence in sequence list, -1 means all")
    parser.add_argument("-l", "--loopclosing", default=1, type=int,
                        help="if 1, enable loop closing")
    parser.add_argument("-t", "--test_id", default=0, type=int, help="test id")
    opt = parser.parse_args()
    print(opt)

    run_orb2(opt)


if __name__ == "__main__":
    main()
