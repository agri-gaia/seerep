#!/usr/bin/env python3

# NOTE: run with 'rosrun seerep_performance_tests run.py'

import glob
import math
import os
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# a file with bytes to use as a message payload
OUTPUT_DIR = Path("/home/pbrstudent/Documents/seerep-data")
MESSAGE_DATA_PATH = Path("/home/pbrstudent/Documents/rosbags/iros/center-rgb8-only.mcap")

CONFIG = {
    # both sizes must be in bytes
    "message_sizes": [1024 * 50, 1024 * 100, 1024**2 * 1, 1024**2 * 10, 1024**2 * 100],
    "total_sizes": [1024**2 * 250, 1024**2 * 500],
}


def convert_bytes(num_bytes: int) -> str:
    if num_bytes == 0:
        return "0B"
    size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
    index = int(math.floor(math.log(num_bytes, 1024)))
    power = math.pow(1024, index)
    size = int(round(num_bytes / power, 2))
    return f"{size}{size_name[index]}"


def build_config() -> dict:
    configs = {}
    for message_size in CONFIG["message_sizes"]:
        for total_size in CONFIG["total_sizes"]:
            label = f"{convert_bytes(message_size)}-{convert_bytes(total_size)}"
            configs[label] = {
                "message_size": message_size,
                "total_size": total_size,
            }
    return configs


def executable_path() -> str:
    output = subprocess.run(
        ["catkin_find", "seerep_performance_tests", "seerep_performance_tests"], check=True, stdout=subprocess.PIPE
    )
    return output.stdout.decode("utf-8").strip()


def run_once(config: dict, label: str) -> None:
    output = subprocess.run(
        [
            executable_path(),
            str(MESSAGE_DATA_PATH),
            label,
            OUTPUT_DIR,
            str(config["message_size"]),
            str(config["total_size"]),
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    print(output.stdout.decode("utf-8").strip())


def cleanup() -> None:
    for file in glob.glob(f"{OUTPUT_DIR}/*.csv"):
        os.remove(file)
    for dir in ["hdf5", "mcap"]:
        path = Path(OUTPUT_DIR / dir)
        if not path.exists():
            path.mkdir()
        else:
            for file in glob.glob(f"{path}/*.{dir}"):
                os.remove(file)


def plot() -> None:
    dfs = []
    for file in glob.glob(f"{OUTPUT_DIR}/*.csv"):
        dfs.append((Path(file).name.split('.')[0], pd.read_csv(file, names=["start", "end", "duration"])))

    # TODO refactor
    mcap, hdf5 = [], []
    sizes = set()
    for label, df in dfs:
        filetype = label.split('-')[0]
        size = "-".join(label.split('-')[1:])
        sizes.add(size)
        if filetype == "mcap":
            mcap.append((size, df.loc[:, "duration"].mean() / 10**9))
        elif filetype == "hdf5":
            hdf5.append((size, df.loc[:, "duration"].mean() / 10**9))

    mcap = sorted(mcap, key=lambda x: x[0])
    hdf5 = sorted(hdf5, key=lambda x: x[0])
    sizes = sorted(sizes)

    bar_width = 0.25

    assert len(mcap) == len(hdf5)

    r1 = np.arange(len(mcap))
    r2 = [x + bar_width for x in r1]

    plt.bar(r1, [val for _, val in mcap], width=bar_width, edgecolor='white', label='mcap', color='#7f6d5f')
    plt.bar(r2, [val for _, val in hdf5], width=bar_width, edgecolor='white', label='hdf5', color='#557f2d')

    plt.xticks([r + (bar_width / 2) for r in range(len(mcap))], sizes, rotation=30, ha="right")
    plt.ylabel("Execution time in [s]")
    plt.xlabel("Message size - Total size")

    plt.tight_layout()
    plt.legend()
    plt.savefig("performance.png")


def main():
    cleanup()
    configs = build_config()
    for label, config in configs.items():
        print(f"Running {label} ...")
        run_once(config, label)
    plot()


if __name__ == "__main__":
    main()
