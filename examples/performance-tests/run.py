#!/usr/bin/env python3

# NOTE: run with 'rosrun seerep_performance_tests run.py'

import glob
import math
import os
import re
import subprocess
from collections import OrderedDict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# a file with bytes to use as a message payload
OUTPUT_DIR = Path("/home/pbrstudent/Documents/seerep-data")
MESSAGE_PAYLOAD = Path("/home/pbrstudent/Documents/rosbags/iros/center-rgb8-only.mcap")

CONFIG = {
    # all sizes must be in bytes !
    "num_runs": 100,
    "message_sizes": [
        1024 * 50,
        1024 * 100,
        1024**2 * 1,
        1024**2 * 10,
    ],
    "total_sizes": [1024**2 * 250],
}


def string_to_bytes(bytes_str: str) -> int:
    UNIT_MAP = {
        "KiB": 1024,
        "MiB": 1024**2,
        "GiB": 1024**3,
        "TiB": 1024**4,
        "PiB": 1024**5,
        "EiB": 1024**6,
        "ZiB": 1024**7,
        "YiB": 1024**8,
    }
    messages_size = bytes_str.split("-")[0]
    unit = messages_size[-3:]
    size = int(messages_size[:-3])
    return size * UNIT_MAP[unit]


def bytes_to_string(num_bytes: int) -> str:
    if num_bytes == 0:
        return "0B"
    size_name = ("B", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB", "YiB")
    index = int(math.floor(math.log(num_bytes, 1024)))
    power = math.pow(1024, index)
    size = int(round(num_bytes / power, 2))
    return f"{size}{size_name[index]}"


def build_config() -> dict:
    configs = {}
    for message_size in CONFIG["message_sizes"]:
        for total_size in CONFIG["total_sizes"]:
            label = f"{bytes_to_string(message_size)}-{bytes_to_string(total_size)}"
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
            str(MESSAGE_PAYLOAD),
            label,
            OUTPUT_DIR,
            str(config["message_size"]),
            str(config["total_size"]),
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    # print(output.stdout.decode("utf-8").strip())


def cleanup_cvs() -> None:
    for file in glob.glob(f"{OUTPUT_DIR}/*.csv"):
        os.remove(file)


def cleanup_data() -> None:
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

    # still messy but now it's sorted!
    runtimes = OrderedDict()
    for label, df in dfs:
        filetype = label.split('-')[0]
        size = "-".join(label.split('-')[1:])
        if not size in runtimes:
            runtimes[size] = {}
        if filetype == "mcap":
            runtimes[size]["mcap"] = {
                "mean": df.loc[:, "duration"].mean() / 10**9,
                "std_dev": df.loc[:, "duration"].std() / 10**9,
            }
        elif filetype == "hdf5":
            runtimes[size]["hdf5"] = {
                "mean": df.loc[:, "duration"].mean() / 10**9,
                "std_dev": df.loc[:, "duration"].std() / 10**9,
            }

    sorted_keys = sorted(runtimes.keys(), key=lambda x: string_to_bytes(x))

    mcap_times, hdf5_times, = (
        [],
        [],
    )
    macp_std_devs, hdf5_std_devs = [], []
    for keys in sorted_keys:
        mcap_times.append((runtimes[keys]["mcap"]["mean"]))
        hdf5_times.append((runtimes[keys]["hdf5"]["mean"]))
        macp_std_devs.append((runtimes[keys]["mcap"]["std_dev"]))
        hdf5_std_devs.append((runtimes[keys]["hdf5"]["std_dev"]))

    assert len(mcap_times) == len(hdf5_times)

    # split the keys into two lines
    sorted_keys = [s.replace("-", "\n") for s in sorted_keys]

    BAR_WIDTH = 0.25
    FONT_SIZE = 12

    # create bar positions
    r1 = np.arange(len(mcap_times))
    r2 = [x + BAR_WIDTH for x in r1]

    fig = plt.figure()
    plt.style.use("seaborn-paper")

    plt.bar(r1, mcap_times, width=BAR_WIDTH, edgecolor="white", label="MCAP", color="#a6a6a6")
    plt.bar(r2, hdf5_times, width=BAR_WIDTH, edgecolor="white", label="HDF5", color="#548235")

    plt.errorbar(r1, mcap_times, yerr=macp_std_devs, fmt="none", capsize=5, capthick=1, ecolor="black")
    plt.errorbar(r2, hdf5_times, yerr=hdf5_std_devs, fmt="none", capsize=5, capthick=1, ecolor="black")

    plt.xticks([r + (BAR_WIDTH / 2) for r in range(len(mcap_times))], sorted_keys, fontsize=FONT_SIZE)
    plt.yticks(fontsize=FONT_SIZE)
    plt.xlabel("Benchmark configuration with message and total written size", labelpad=10, fontsize=FONT_SIZE)
    plt.ylabel("Execution time in seconds", fontsize=FONT_SIZE)

    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.legend(prop={"size": FONT_SIZE})
    # plt.show()
    plt.savefig(Path(OUTPUT_DIR / "results.png"), dpi=600)


def main():
    cleanup_cvs()
    configs = build_config()
    for label, config in configs.items():
        print(f"Running {label} ...")
        for run in range(CONFIG["num_runs"]):
            print(f"Run {run + 1} ...")
            cleanup_data()
            run_once(config, label)
    print("Generating plot ...")
    plot()


if __name__ == "__main__":
    main()
