import os
import os.path as op
import subprocess
import argparse
import glob
import time

PKG_NAME = "rovioli"
NODE_NAME = "run_rovioli_onlyvo"
BAG_ROOT = "/dataset"
OUTPUT_ROOT = "/work/output"
TEST_NUM = 5

# maplab USAGE:
# rosrun rovioli run_rovioli_scratch
#   /data/euroc_mav/MH_01_easy/MH_01_easy.bag
#   /work/output/euroc_mav_mh01/traj.csv


def run_maplab(opt):
    check_base_paths()

    if opt.dataset == "all":
        command_makers = [euroc_mav]
        commands = []
        configs = []
        for preset in [0, 1]:
            opt.preset = preset
            for tid in range(TEST_NUM):
                opt.test_id = tid
                for cmdmaker in command_makers:
                    cmds, cfgs = cmdmaker(opt)
                    commands.extend(cmds)
                    configs.extend(cfgs)
    elif opt.dataset == "euroc_mav":
        commands, configs = euroc_mav(opt)
    else:
        raise FileNotFoundError()

    # return
    for i in range(5):
        print("start maplab in {} sec".format(5-i))
        time.sleep(1)

    for cmd, cfg in zip(commands, configs):
        outfile = cmd[-1][7:]
        os.makedirs(op.dirname(outfile), exist_ok=True)
        print("\n===== RUN DSO\nconfig: {}\ncmd: {}\n".format(cfg, cmd))
        subprocess.run(cmd)
        subprocess.run(["chmod", "-R", "a+rw", OUTPUT_ROOT])
        assert op.isfile(outfile), "===== ERROR: output file was NOT created: {}".format(outfile)


def check_base_paths():
    assert op.isfile("/work/catkin_ws/devel/lib/rovioli/rovioli"), "ROVIOLI executer doesn't exist"
    assert op.isdir(BAG_ROOT), "datset dir doesn't exist"
    assert op.isdir(OUTPUT_ROOT), "output dir doesn't exist"


def euroc_mav(opt):
    dataset_path = op.join(BAG_ROOT, "euroc_bag")
    outname = "rovioli_vio"
    sequences = glob.glob(dataset_path + "*.bag")
    if opt.seq_idx != -1:
        sequences = [sequences[opt.seq_idx]]

    commands = []
    configs = []
    for si, bagfile in enumerate(sequences):
        outfile = op.basename(bagfile)
        outfile = outfile.split("_")
        outfile = op.join(OUTPUT_ROOT, outname, "{}_{}.csv".format(outfile[0], outfile[1]))

        assert op.isfile(bagfile), "bag doesn't exist: " + bagfile

        cmd = ["rosrun", PKG_NAME, NODE_NAME, bagfile, outfile]
        print("===== tum_mono_vo command =====\n", " ".join(cmd))
        commands.append(cmd)
        conf = {"dataset": "euroc_mav", "sequence": op.basename(bagfile),
                "test id": opt.test_id, "output": outfile}
        configs.append(conf)
    return commands, configs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=str, help="dataset name")
    parser.add_argument("-t", "--test_id", default=0, type=int, help="test id")
    parser.add_argument("-s", "--seq_idx", default=-1, type=int,
                        help="int: index of sequence in sequence list, -1 means all")
    opt = parser.parse_args()

    run_maplab(opt)


if __name__ == "__main__":
    main()
