import os
import os.path as op
import subprocess
import argparse
import glob
import sequence_abbrev as sa


class RunORB2:
    def __init__(self, opt):
        self.ORB2_ROOT = "/work/ORB_SLAM2"
        self.VOCABULARY = self.ORB2_ROOT + "/Vocabulary/ORBvoc.txt"
        self.DATA_ROOT = "/data/dataset"
        self.OUTPUT_ROOT = "/data/output/pose"
        self.TEST_IDS = list(range(opt.num_test))

    def run_orb2(self, opt):
        self.check_base_paths()
        commands, configs = self.generate_commands(opt)
        self.execute_commands(commands, configs)

    def check_base_paths(self):
        assert op.isdir(self.ORB2_ROOT), "ORB SLAM2 dir doesn't exist"
        assert op.isfile(self.VOCABULARY), "Vocabulary file doesn't exist"
        assert op.isdir(self.DATA_ROOT), "dataset dir doesn't exist"
        assert op.isdir(self.OUTPUT_ROOT), "output dir doesn't exist"

    def generate_commands(self, opt):
        if opt.exec == "all":
            command_makers = [self.stereo_tumvi, self.stereo_euroc, self.stereo_kitti]
            commands = []
            configs = []
            for lc in [0]:
                opt.loopclosing = lc
                for cmdmaker in command_makers:
                    cmds, cfgs = cmdmaker(opt)
                    commands.extend(cmds)
                    configs.extend(cfgs)
        elif opt.exec == "stereo_kitti":
            commands, configs = self.stereo_kitti(opt)
        elif opt.exec == "stereo_euroc":
            commands, configs = self.stereo_euroc(opt)
        elif opt.exec == "stereo_tumvi":
            commands, configs = self.stereo_tumvi(opt)
        else:
            raise FileNotFoundError()

        print("\n===== Total {} runs\n".format(len(commands)))
        return commands, configs

    def execute_commands(self, commands, configs):
        for ci, (cmd, cfg) in enumerate(zip(commands, configs)):
            outfile = cmd[-1]
            os.makedirs(op.dirname(outfile), exist_ok=True)

            print("\n===== RUN ORB_SLAM2 {}/{}\nconfig: {}\ncmd: {}\n"
                  .format(ci+1, len(commands), cfg, cmd))
            if op.isfile(outfile):
                print("This config has already executed, skip it ....")
                continue

            subprocess.run(cmd)
            subprocess.run(["chmod", "-R", "a+rw", self.OUTPUT_ROOT])
            assert op.isfile(outfile), "===== ERROR: output file was NOT created: {}".format(outfile)

    # Usage:
    # ./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt \
    #       Examples/Stereo/KITTIX.yaml \
    #       PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER \
    #       loop_closing_on \
    #       output_file
    def stereo_kitti(self, opt):
        exec_path = op.join(self.ORB2_ROOT, "Examples/Stereo")
        executer = op.join(exec_path, "stereo_kitti")
        dataset = "kitti_odometry"
        dataset_path = op.join(self.DATA_ROOT, dataset, "sequences")
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        sequences = [s.rstrip("/") for s in glob.glob(dataset_path + "/*/")]
        sequences = [s for s in sequences if int(op.basename(s)) <= 10]
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        sequences.sort()
        outname = "orb2_vo_stereo" if opt.loopclosing == 0 else "orb2_slam_stereo"

        commands = []
        configs = []
        for si, seq_path in enumerate(sequences):
            for test_id in self.TEST_IDS:
                seq_id = int(op.basename(seq_path))
                if seq_id < 3:
                    config_file = op.join(exec_path, "KITTI00-02.yaml")
                elif seq_id == 3:
                    config_file = op.join(exec_path, "KITTI03.yaml")
                elif seq_id > 3:
                    config_file = op.join(exec_path, "KITTI04-12.yaml")
                else:
                    raise FileNotFoundError()

                output_file = op.join(output_path, "{}_s{:02d}_{}.txt".format(outname, si, test_id))
                cmd = [executer, self.VOCABULARY, config_file, seq_path, str(opt.loopclosing), output_file]
                commands.append(cmd)
                conf = {"executer": outname, "loop closing": opt.loopclosing, "dataset": dataset,
                        "sequence": si, "test id": test_id}
                configs.append(conf)
            print("===== command: ", " ".join(commands[-1]))
        return commands, configs

    # Usage:
    # ./Examples/Stereo/streo_euroc Vocabulary/ORBvoc.txt \
    #       Examples/Stereo/EuRoC.yaml \
    #       PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data \
    #       PATH_TO_SEQUENCE_FOLDER/mav0/cam1/data \
    #       Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt \
    #       loop_closing_on \
    #       path/to/output/file
    def stereo_euroc(self, opt):
        exec_path = op.join(self.ORB2_ROOT, "Examples/Stereo")
        executer = op.join(exec_path, "stereo_euroc")
        dataset = "euroc_mav"
        dataset_path = op.join(self.DATA_ROOT, dataset)
        sequences = [s + "mav0/cam0/data" for s in glob.glob(dataset_path + "/*/") if op.isdir(s+"mav0")]
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        sequences.sort()
        outname = "orb2_vo_stereo" if opt.loopclosing == 0 else "orb2_slam_stereo"

        config_file = op.join(exec_path, "EuRoC.yaml")
        commands = []
        configs = []
        for si, cam0_seq in enumerate(sequences):
            cam1_seq = cam0_seq.replace("cam0", "cam1")
            timefile = cam0_seq.split("/")[-4]
            timefile = timefile.split("_")
            timefile = op.join(exec_path, "EuRoC_TimeStamps", timefile[0] + timefile[1] + ".txt")
            seq_abbr = sa.euroc_abbrev(cam0_seq.split("/")[-4])
            for test_id in self.TEST_IDS:
                output_file = op.join(output_path, "{}_{}_{}.txt".format(outname, seq_abbr, test_id))
                cmd = [executer, self.VOCABULARY, config_file, cam0_seq, cam1_seq, timefile,
                       str(opt.loopclosing), output_file]
                commands.append(cmd)
                conf = {"executer": outname, "loop closing": opt.loopclosing, "dataset": dataset,
                        "sequence": seq_abbr, "test id": test_id}
                configs.append(conf)
            print("===== command:", " ".join(commands[-1]))
        return commands, configs

    # Usage:
    # ./Examples/Stereo/streo_euroc Vocabulary/ORBvoc.txt \
    #       Examples/Stereo/TumVI.yaml \
    #       PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data \
    #       PATH_TO_SEQUENCE_FOLDER/mav0/cam1/data \
    #       Examples/Stereo/TumVI_TimeStamps/SEQUENCE.txt \
    #       loop_closing_on \
    #       path/to/output/file
    def stereo_tumvi(self, opt):
        exec_path = op.join(self.ORB2_ROOT, "Examples/Stereo")
        executer = op.join(exec_path, "stereo_euroc")
        dataset = "tum_vi"
        dataset_path = op.join(self.DATA_ROOT, dataset)
        sequences = [s + "mav0/cam0/data" for s in glob.glob(dataset_path + "/*/") if op.isdir(s+"mav0")]
        output_path = op.join(self.OUTPUT_ROOT, dataset)
        if opt.seq_idx != -1:
            sequences = [sequences[opt.seq_idx]]
        sequences.sort()
        outname = "orb2_vo_stereo" if opt.loopclosing == 0 else "orb2_slam_stereo"

        config_file = op.join(exec_path, "TumVI.yaml")
        commands = []
        configs = []
        for si, cam0_seq in enumerate(sequences):
            cam1_seq = cam0_seq.replace("cam0", "cam1")
            timefile = cam0_seq.split("/")[-4]
            timefile = op.join(exec_path, "TumVI_TimeStamps", timefile + ".txt")
            seq_abbr = sa.tumvi_abbrev(cam0_seq.split("/")[-4])
            for test_id in self.TEST_IDS:
                output_file = op.join(output_path, "{}_{}_{}.txt".format(outname, seq_abbr, test_id))
                cmd = [executer, self.VOCABULARY, config_file, cam0_seq, cam1_seq, timefile,
                       str(opt.loopclosing), output_file]
                commands.append(cmd)
                conf = {"executer": outname, "loop closing": opt.loopclosing, "dataset": dataset,
                        "sequence": seq_abbr, "test id": test_id}
                configs.append(conf)
            print("===== command:", " ".join(commands[-1]))
        return commands, configs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exec", default="all", type=str, help="orbslam2 executer")
    parser.add_argument("-l", "--loopclosing", default=0, type=int,
                        help="if 1, enable loop closing")
    parser.add_argument("-s", "--seq_idx", default=-1, type=int,
                        help="int: index of sequence in sequence list, -1 means all")
    parser.add_argument("-t", "--num_test", default=5, type=int,
                        help="number of tests per sequence")
    opt = parser.parse_args()
    print(opt)

    orbslam2 = RunORB2(opt)
    orbslam2.run_orb2(opt)


if __name__ == "__main__":
    main()
