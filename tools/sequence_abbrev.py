import os.path as op


def euroc_abbrev(seq_name):
    seq_name = seq_name.split("_")
    seq_name = op.join(seq_name[0] + seq_name[1])
    return seq_name


def tumvi_abbrev(seq_name):
    print(seq_name)
    seq_abbr = seq_name.split("-")[1].split("_")[0]
    return seq_abbr
