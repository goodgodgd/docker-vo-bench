import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import settings
from define_paths import *


def convert_euroc():
    seq_names = [s.rstrip("/") for s in glob.glob(EUROC_PATH + "/*/") if op.isdir(s + "mav0")]
    seq_gt_files = [s + "mav0/state_groundtruth_estimate0/data.csv"
                    for s in glob.glob(EUROC_PATH + "/*/") if op.isdir(s + "mav0")]
    for sname, gtfile in zip(seq_names, seq_gt_files):
        data = pd.read_csv(gtfile)
        out_columns = [col for col in list(data)
                       if col.startswith("#time") or "p_" in col or "q_" in col]
        outdata = data[out_columns]
        outfile = op.join(OUTPUT_PATH, "ground_truth", "euroc_mav", sname + ".csv")
        assert op.isdir(OUTPUT_PATH), "output root does NOT exist: {}".format(OUTPUT_PATH)
        os.makedirs(op.basename(outfile), exist_ok=True)
        outdata.to_csv(outfile, encoding="utf-8")


if __name__ == "__main__":
    convert_euroc()
