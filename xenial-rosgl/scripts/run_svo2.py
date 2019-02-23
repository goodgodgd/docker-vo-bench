import os
import os.path as op
import subprocess
import argparse
import glob
import time
import sequence_abbrev as sa


class RunSVO2:
    def __init__(self, opt):
        self.PKG_NAME = "svo_ros"
        self.DATA_ROOT = "/data/dataset"
        self.OUTPUT_ROOT = "/data/output"
        self.TEST_IDS = list(range(opt.num_test))

    def run_svo2(self, opt):
        self.check_base_paths()
        commands, configs = self.generate_commands(opt)
        self.execute_commands(commands, configs)

    def check_base_paths(self):
        assert op.isdir(self.DATA_ROOT), "datset dir doesn't exist"
        assert op.isdir(self.OUTPUT_ROOT), "output dir doesn't exist"

    def generate_commands(self, opt):
        if opt.dataset == "all":
            command_makers = [self.tum_vi, self.euroc_mav]
            commands = []
            configs = []
            for cmdmaker in command_makers:
                cmds, cfgs = cmdmaker(opt)
                commands.extend(cmds)
                configs.extend(cfgs)
        elif opt.dataset == "euroc":
            commands, configs = self.euroc_mav(opt)
        elif opt.dataset == "tumvi":
            commands, configs = self.tum_vi(opt)
        else:
            raise FileNotFoundError()

        print("\n===== Total {} runs\n".format(len(commands)))
        return commands, configs

    def execute_commands(self, commands, configs):
        for i in range(3):
            print("start SVO2 in {} sec".format(3-i))
            time.sleep(1)

        subprocess.Popen(["roscore"])
        time.sleep(3)
        for ci, (cmd, cfg) in enumerate(zip(commands, configs)):
            bagfile = cmd[0]
            outfile = cmd[-1]
            outfile = outfile[9:]
            print("output path:", op.dirname(outfile))
            os.makedirs(op.dirname(outfile), exist_ok=True)

            print("\n===== RUN SVO2 {}/{}\nconfig: {}\ncmd: {}\n"
                  .format(ci+1, len(commands), cfg, cmd))
            if op.isfile(outfile):
                print("This config has already executed, skip it ....")
                continue

            subprocess.Popen(cmd[1:])
            time.sleep(10)
            subprocess.run(["rosbag", "play", bagfile],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(5)
            subprocess.run(["chmod", "-R", "a+rw", self.OUTPUT_ROOT])
            assert op.isfile(outfile), "===== ERROR: output file was NOT created: {}".format(outfile)
        subprocess.run(["pkill", "roscore"])

    # Usage:
    # roslaunch svo_ros xxxxx.launch outfile:=/path/to/output
    def euroc_mav(self, opt):
        dataset = "euroc_mav"
        dataset_path = op.join(self.DATA_ROOT, dataset, "bags")
        launch_files = {"mvio": "euroc_mono_imu.launch",
                        "svio": "euroc_stereo_imu.launch",
                        "stereo": "euroc_stereo.launch"}
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        if not op.isdir(output_path):
            os.makedirs(output_path)
        sequences = glob.glob(dataset_path + "/*.bag")
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        sequences.sort()
        outprefix = "svo2"

        commands = []
        configs = []
        for suffix, launch in launch_files.items():
            for si, bagfile in enumerate(sequences):
                outname = outprefix + "_" + suffix
                seq_abbr = sa.euroc_abbrev(bagfile.split("/")[-1])
                for test_id in self.TEST_IDS:
                    output_file = op.join(output_path, "{}_{}_{}.txt".format(outname, seq_abbr, test_id))
                    cmd = [bagfile, "roslaunch", self.PKG_NAME, launch, "outfile:=" + output_file]
                    commands.append(cmd)
                    conf = {"executer": outname, "launch": launch, "dataset": dataset,
                            "seq_name": op.basename(bagfile), "seq_id": si, "test id": test_id}
                    configs.append(conf)
                print("===== command:", " ".join(commands[-1]))
        return commands, configs

    # Usage:
    # roslaunch svo_ros xxxxx.launch outfile:=/path/to/output
    def tum_vi(self, opt):
        dataset = "tum_vi"
        dataset_path = op.join(self.DATA_ROOT, dataset, "bags")
        launch_files = {"mvio": "tumvi_mono_imu.launch",
                        "svio": "tumvi_stereo_imu.launch",
                        "stereo": "tumvi_stereo.launch"}
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        if not op.isdir(output_path):
            os.makedirs(output_path)
        sequences = glob.glob(dataset_path + "/*.bag")
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        sequences.sort()
        outprefix = "svo2"

        commands = []
        configs = []
        for suffix, launch in launch_files.items():
            for si, bagfile in enumerate(sequences):
                outname = outprefix + "_" + suffix
                seq_abbr = sa.tumvi_abbrev(bagfile.split("/")[-1])
                for test_id in self.TEST_IDS:
                    output_file = op.join(output_path, "{}_{}_{}.txt".format(outname, seq_abbr, test_id))
                    cmd = [bagfile, "roslaunch", self.PKG_NAME, launch, "outfile:=" + output_file]
                    commands.append(cmd)
                    conf = {"executer": outname, "launch": launch, "dataset": dataset,
                            "seq_name": op.basename(bagfile), "seq_id": si, "test id": test_id}
                    configs.append(conf)
                print("===== command:", " ".join(commands[-1]))
        return commands, configs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset", default="all", type=str, help="dataset name")
    parser.add_argument("-t", "--num_test", default=5, type=int, help="number of tests per sequence")
    parser.add_argument("-s", "--seq_idx", default=-1, type=int,
                        help="int: index of sequence in sequence list, -1 means all")
    opt = parser.parse_args()

    svo2 = RunSVO2(opt)
    svo2.run_svo2(opt)


if __name__ == "__main__":
    main()
