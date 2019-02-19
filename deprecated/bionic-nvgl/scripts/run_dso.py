import os
import os.path as op
import subprocess
import argparse
import glob
import time

EXECUTER = "/work/project/dso/build/bin/dso_dataset"
DATA_ROOT = "/work/dataset"
OUTPUT_ROOT = "/work/output"
TEST_NUM = 5
TEST_IDS = None


def run_dso(opt):
    check_base_paths()
    global TEST_IDS
    TEST_IDS = list(range(TEST_NUM)) if opt.test_id < 0 else [opt.test_id]

    if opt.dataset == "all":
        command_makers = [tum_mono_vo, euroc_mav]
        commands = []
        configs = []
        for preset in [0, 1]:
            opt.preset = preset
            for cmdmaker in command_makers:
                cmds, cfgs = cmdmaker(opt)
                commands.extend(cmds)
                configs.extend(cfgs)
    elif opt.dataset == "tum_mono_vo":
        commands, configs = tum_mono_vo(opt)
    elif opt.dataset == "euroc_mav":
        commands, configs = euroc_mav(opt)
    else:
        raise FileNotFoundError()

    # return
    for i in range(5):
        print("start DSO in {} sec".format(5-i))
        time.sleep(1)

    for ci, (cmd, cfg) in enumerate(zip(commands, configs)):
        outfile = cmd[-1]
        os.makedirs(op.dirname(outfile), exist_ok=True)
        print("\n===== RUN DSO {}/{}\nconfig: {}\ncmd: {}\n"
              .format(ci+1, len(commands), cfg, cmd))
        subprocess.run(cmd)
        subprocess.run(["chmod", "-R", "a+rw", OUTPUT_ROOT])
        assert op.isfile(outfile), "===== ERROR: output file was NOT created: {}".format(outfile)


def check_base_paths():
    assert op.isfile(EXECUTER), "DSO executer doesn't exist"
    assert op.isdir(DATA_ROOT), "datset dir doesn't exist"
    assert op.isdir(OUTPUT_ROOT), "output dir doesn't exist"


# DSO usage
# bin / dso_dataset \
#     files = XXXXX / sequence_XX / images.zip \
#     calib = XXXXX / sequence_XX / camera.txt \
#     gamma = XXXXX / sequence_XX / pcalib.txt \
#     vignette = XXXXX / sequence_XX / vignette.png \
#     preset = 0 \
#     mode = 0 \
#     result = XXXXX

# preset=0: default settings (2k pts etc.), not enforcing real-time execution
# preset=1: default settings (2k pts etc.), enforcing 1x real-time execution
# WARNING: preset=2 or 3 overwrites image resolution with 424 x 320.
# preset=2: fast settings (800 pts etc.), not enforcing real-time execution.
# preset=3: fast settings (800 pts etc.), enforcing 5x real-time execution

# mode=0 use iff a photometric calibration exists (e.g. TUM monoVO dataset).
# mode=1 use iff NO photometric calibration exists (e.g. ETH EuRoC MAV dataset).
# mode=2 use iff images are not photometrically distorted (e.g. synthetic datasets).

def tum_mono_vo(opt):
    dataset_path = op.join(DATA_ROOT, "tum_mono_vo")
    mode = "0"
    outname = "dso_efrt" if opt.preset == 0 else "dso_nort"
    output_path = op.join(OUTPUT_ROOT, outname, "pose")
    if not op.isdir(output_path):
        os.makedirs(output_path)
    sequences = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/")]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    commands = []
    configs = []
    for si, seq_path in enumerate(sequences):
        for test_id in TEST_IDS:
            image = op.join(seq_path, "images.zip")
            calib = op.join(seq_path, "camera.txt")
            gamma = op.join(seq_path, "pcalib.txt")
            vignette = op.join(seq_path, "vignette.png")
            preset = str(opt.preset)
            filename = "tum_mono_s{}_t{}.txt".format(op.basename(seq_path)[-2:], test_id)
            result = op.join(output_path, filename)

            cmd = [EXECUTER, "files=" + image, "calib=" + calib, "gamma=" + gamma,
                   "vignette=" + vignette, "preset=" + preset, "mode=" + mode,
                   "quiet=1", "result=" + result]
            print("===== tum_mono_vo command =====\n", " ".join(cmd))
            commands.append(cmd)
            conf = {"dataset": "tum_mono_vo", "sequence": op.basename(seq_path),
                    "preset": preset, "mode": mode, "test id": test_id}
            configs.append(conf)
    return commands, configs


def euroc_mav(opt):
    dataset_path = op.join(DATA_ROOT, "euroc")
    mode = "1"
    outname = "dso_efrt" if opt.preset == 0 else "dso_nort"
    output_path = op.join(OUTPUT_ROOT, outname, "pose")
    if not op.isdir(output_path):
        os.makedirs(output_path)
    sequences = [s + "mav0/cam0" for s in glob.glob(dataset_path + "/*/")]
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    commands = []
    configs = []
    for si, seq_path in enumerate(sequences):
        for test_id in TEST_IDS:
            create_euroc_calib(seq_path)
            image = op.join(seq_path, "data")
            calib = op.join(seq_path, "camera.txt")
            preset = str(opt.preset)
            filename = "euroc_mav_s{}_t{}.txt".format(si, test_id)
            result = op.join(output_path, filename)

            cmd = [EXECUTER, "files=" + image, "calib=" + calib, "preset=" + preset,
                   "mode=" + mode, "quiet=1", "result=" + result]
            print("===== tum_mono_vo command =====\n", " ".join(cmd))
            commands.append(cmd)
            conf = {"dataset": "tum_mono_vo", "sequence": op.basename(seq_path),
                    "preset": preset, "mode": mode, "test id": test_id}
            configs.append(conf)
    return commands, configs


def create_euroc_calib(seq_path):
    # to use yaml package, run
    # apt install python3-pip && pip3 install ruamel.yaml
    from ruamel import yaml
    f_yaml = open(op.join(seq_path, "sensor.yaml"), "r")
    data = yaml.safe_load(f_yaml)

    intrinsics = data["intrinsics"]
    resolution = data["resolution"]
    distortion = data["distortion_coefficients"]

    # Note: fx fy cx cy are relative to the image width / height
    intrinsics[0] /= resolution[0]  # fx
    intrinsics[1] /= resolution[1]  # fy
    intrinsics[2] /= resolution[0]  # cx
    intrinsics[3] /= resolution[1]  # cy
    calibrated = [round(intrinsics[0]*100.)/100., round(intrinsics[1]*100.)/100., 0.5, 0.5]

    # camera.txt format
    # RadTan fx fy cx cy k1 k2 r1 r2
    # in_width in_height
    # "crop" / "full" / "fx fy cx cy 0"
    # out_width out_height
    f_cam = open(op.join(seq_path, "camera.txt"), "w")
    params = intrinsics + distortion
    f_cam.write("RadTan " + ("{:.6f} "*len(params)).format(*params).rstrip() + "\n")
    f_cam.write(" ".join(str(res) for res in resolution) + "\n")
    f_cam.write(" ".join(str(par) for par in calibrated) + " 0\n")
    f_cam.write(" ".join(str(res) for res in resolution) + "\n")
    f_cam.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=str, help="dataset name")
    parser.add_argument("-m", "--preset", default=0, type=int,
                        help="0: not enforcing realtime, 1: enforcing realtime")
    parser.add_argument("-t", "--test_id", default=-1, type=int, help="test id")
    parser.add_argument("-s", "--seq_idx", default=-1, type=int,
                        help="int: index of sequence in sequence list, -1 means all")
    opt = parser.parse_args()

    run_dso(opt)


if __name__ == "__main__":
    main()
