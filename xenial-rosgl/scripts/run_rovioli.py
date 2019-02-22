import os
import os.path as op
import subprocess
import argparse
import glob
import time
import pandas as pd
import numpy as np


class RunROVIOLI:
    def __init__(self):
        self.PKG_NAME = "rovioli"
        self.DATA_ROOT = "/data/dataset"
        self.OUTPUT_ROOT = "/data/output"
        self.NUM_TEST = 5
        self.TEST_IDS = None

    def run_rovioli(self, opt):
        self.check_base_paths()
        self.TEST_IDS = list(range(self.NUM_TEST)) if opt.test_id < 0 else [opt.test_id]
        commands, configs = self.generate_commands(opt)
        self.execute_commands(commands, configs)

    def check_base_paths(self):
        assert op.isdir(self.DATA_ROOT), "datset dir doesn't exist"
        assert op.isdir(self.OUTPUT_ROOT), "output dir doesn't exist"

    def generate_commands(self, opt):
        if opt.dataset == "all":
            command_makers = [self.euroc_mav, self.tum_vi]
            commands = []
            configs = []
            for cmdmaker in command_makers:
                cmds, cfgs = cmdmaker(opt)
                commands.extend(cmds)
                configs.extend(cfgs)
        elif opt.dataset == "euroc_mav":
            commands, configs = self.euroc_mav(opt)
        else:
            raise FileNotFoundError()

        print("\n===== Total {} runs\n".format(len(commands)))
        return commands, configs

    def execute_commands(self, commands, configs):
        for i in range(3):
            print("start maplab in {} sec".format(3-i))
            time.sleep(1)

        subprocess.Popen(["roscore"])
        time.sleep(3)
        for ci, (cmd, cfg) in enumerate(zip(commands, configs)):
            outfile = cmd[-1]
            os.makedirs(op.dirname(outfile), exist_ok=True)

            print("\n===== RUN ROVIOLI {}/{}\nconfig: {}\ncmd: {}\n"
                  .format(ci+1, len(commands), cfg, cmd))
            txtfile = outfile.replace(".csv", ".txt")
            if op.isfile(txtfile):
                print("This config has already executed, skip it ....")
                continue

            subprocess.run(cmd)
            subprocess.run(["chmod", "-R", "a+rw", self.OUTPUT_ROOT])
            assert op.isfile(outfile), "===== ERROR: output file was NOT created: {}".format(outfile)
            self.format_tum_and_savetxt(outfile)

    @staticmethod
    def format_tum_and_savetxt(csvfile):
        data = pd.read_csv(csvfile)
        data = data.values
        os.remove(csvfile)
        if data.shape[1] > 8:
            data = np.concatenate([np.expand_dims(data[:, 0], 1), data[:, 8:15]], axis=1)
            txtfile = csvfile.replace(".csv", ".txt")
            np.savetxt(txtfile, data, fmt="%1.6f")

    # Usage:
    # rosrun rovioli run_rovioli_scratch
    #       /path/to/dataset/MH_01_easy.bag
    #       output_file
    def euroc_mav(self, opt):
        node_name = "run_rovioli_euroc_vo"
        dataset = "euroc_mav"
        dataset_path = op.join(self.DATA_ROOT, dataset, "bags")
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        if not op.isdir(output_path):
            os.makedirs(output_path)
        sequences = glob.glob(dataset_path + "/*.bag")
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        sequences.sort()
        outname = "rovioli_mvio"

        commands = []
        configs = []
        for si, bagfile in enumerate(sequences):
            for test_id in self.TEST_IDS:
                output_file = op.join(output_path, "{}_s{:02d}_{}.txt".format(outname, si, test_id))

                cmd = ["rosrun", self.PKG_NAME, node_name, bagfile, output_file]
                commands.append(cmd)
                conf = {"executer": outname, "dataset": dataset, "seq_name": op.basename(bagfile),
                        "seq_id": si, "test_id": test_id}
                configs.append(conf)
            print("===== command:", " ".join(commands[-1]))
        return commands, configs

    # Usage:
    # rosrun rovioli run_rovioli_scratch
    #       /path/to/dataset/MH_01_easy.bag
    #       output_file
    def tum_vi(self, opt):
        node_name = "run_rovioli_tumvi_vo"
        dataset = "tum_vi"
        dataset_path = op.join(self.DATA_ROOT, dataset, "bags")
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        if not op.isdir(output_path):
            os.makedirs(output_path)
        sequences = glob.glob(dataset_path + "/*.bag")
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        outname = "rovioli_mvio"

        commands = []
        configs = []
        for si, bagfile in enumerate(sequences):
            for test_id in self.TEST_IDS:
                output_file = op.join(output_path, "{}_s{:02d}_{}.csv".format(outname, si, test_id))

                cmd = ["rosrun", self.PKG_NAME, node_name, bagfile, output_file]
                commands.append(cmd)
                conf = {"executer": outname, "dataset": dataset, "seq_name": op.basename(bagfile),
                        "seq_id": si, "test_id": test_id}
                configs.append(conf)
            print("===== command:", " ".join(commands[-1]))
        return commands, configs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset", default="all", type=str, help="dataset name")
    parser.add_argument("-t", "--test_id", default=-1, type=int, help="test id")
    parser.add_argument("-s", "--seq_idx", default=-1, type=int,
                        help="int: index of sequence in sequence list, -1 means all")
    opt = parser.parse_args()

    rovioli = RunROVIOLI()
    rovioli.run_rovioli(opt)


if __name__ == "__main__":
    main()
