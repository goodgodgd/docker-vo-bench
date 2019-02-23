import os
import os.path as op
import glob
import settings
from define_paths import *


def create_timestamp():
    sequences = os.listdir(TUMVI_PATH)
    for seq_name in sequences:
        print("seq_name:", seq_name)
        srcpath = op.join(TUMVI_PATH, seq_name, "mav0/cam0/data")
        dstpath = op.join(ORB2_PATH, "Examples/Stereo/TumVI_TimeStamps")
        dstfile = op.join(dstpath, seq_name + ".txt")

        timestamps = glob.glob(srcpath + "/*.png")
        timestamps = [op.basename(time)[:-4] for time in timestamps]
        timestamps.sort()
        timestamps = [time + "\n" for time in timestamps]
        print("times", timestamps[:10])
        with open(dstfile, 'w') as f:
            f.writelines(timestamps)


if __name__ == "__main__":
    create_timestamp()
