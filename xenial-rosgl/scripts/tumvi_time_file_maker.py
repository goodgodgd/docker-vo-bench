import os.path as op
import glob


def create_timestamp():
    srcpath = "/data/dataset/tum_vi/dataset-room3_512_16/dso/cam0/images"
    dstpath = "/work/ORB_SLAM2/Examples/Stereo/TumVI_TimeStamps"
    dataset_name = srcpath.split("/")[-4]
    dstfile = op.join(dstpath, dataset_name + ".txt")

    timestamps = glob.glob(srcpath + "/*.png")
    timestamps = [op.basename(time)[:-4] for time in timestamps]
    timestamps.sort()
    timestamps = [time + "\n" for time in timestamps]
    print("times", timestamps[:10])
    with open(dstfile, 'w') as f:
        f.writelines(timestamps)


if __name__ == "__main__":
    create_timestamp()
