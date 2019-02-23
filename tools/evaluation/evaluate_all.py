import os
import os.path as op
import numpy as np
import pandas as pd
import glob
import settings
from define_paths import *

algo_prefixes = ["orb2_vo_stereo", "rovioli_mvio",
                 "vinsfs_mvio", "vinsfs_svid", "vinsfx_stereo",
                 "svo2_mvio", "svo2_svid", "svo2_stereo"]


def create_ate_table(dataset):
    result_path = op.join(OUTPUT_PATH, dataset)
    gtruth_path = op.join(OUTPUT_PATH, "ground_truth", dataset)
    


def main():
    table = create_ate_table("euroc_mav")


if __name__ == "__main__":
    main()
