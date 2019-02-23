import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import settings
from define_paths import *

# Note: Set EUROC_PATH, OUTPUT_PATH in defins_paths.py before run this


def convert_euroc(srcpath, dstpath):
    dataset = "euroc_mav"
    seq_names = [op.basename(s.rstrip("/"))
                 for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    seq_gt_files = [s + "mav0/state_groundtruth_estimate0/data.csv"
                    for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    for sname, gtfile in zip(seq_names, seq_gt_files):
        data = pd.read_csv(gtfile)

        # extract required columns: timestamp, x, y, z, qx, qy, qz, qw
        out_columns = [col for col in list(data)
                       if col.startswith("#time") or "p_" in col or "q_" in col]
        outdata = data[out_columns]

        # rename columns
        new_names = ["#timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
        renamer = dict(zip(out_columns, new_names))
        outdata = outdata.rename(columns=renamer)

        # nanosecond timestamp to second
        outdata["#timestamp"] = outdata["#timestamp"] / 1.0E+9
        outdata = outdata.set_index("#timestamp")

        # save file
        savefile = op.join(dstpath, "ground_truth", dataset, sname + ".csv")
        print("save to", savefile)
        print(outdata.head())
        assert op.isdir(dstpath), "output root does NOT exist: {}".format(dstpath)
        os.makedirs(op.dirname(savefile), exist_ok=True)
        outdata.to_csv(savefile, encoding="utf-8", float_format="%1.06f")


def convert_tumvi(srcpath, dstpath):
    dataset = "tum_vi"
    seq_names = [op.basename(s.rstrip("/"))
                 for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    seq_gt_files = [s + "mav0/mocap0/data.csv"
                    for s in glob.glob(srcpath + "/*/") if op.isdir(s + "mav0")]
    for sname, gtfile in zip(seq_names, seq_gt_files):
        data = pd.read_csv(gtfile)

        # rename columns
        new_names = ["#timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
        renamer = dict(zip(list(data), new_names))
        data = data.rename(columns=renamer)

        # nanosecond timestamp to second
        data["#timestamp"] = data["#timestamp"] / 1.0E+9
        data = data.set_index("#timestamp")

        # save file
        seqname = sname.split("-")[1].split("_")[0]
        savefile = op.join(dstpath, "ground_truth", dataset, seqname + ".csv")
        print("save to", savefile)
        print(data.head())
        assert op.isdir(dstpath), "output root does NOT exist: {}".format(dstpath)
        os.makedirs(op.dirname(savefile), exist_ok=True)
        data.to_csv(savefile, encoding="utf-8", float_format="%1.09f")


if __name__ == "__main__":
    convert_tumvi(TUMVI_PATH, OUTPUT_PATH)
    convert_euroc(EUROC_PATH, OUTPUT_PATH)
