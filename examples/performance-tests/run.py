#!/usr/bin/env python3

# NOTE: run with 'rosrun seerep_performance_tests run.py'

import glob
import math
import os
import subprocess
from pathlib import Path

# a file with bytes to use as a message payload
OUTPUT_DIR = Path("/home/pbrstudent/Documents/seerep-data")
MESSAGE_DATA_PATH = Path("/home/pbrstudent/Documents/rosbags/iros/center-rgb8-only.mcap")

CONFIG = {
    # both sizes must be in bytes
    "message_sizes": [1024**2 * 100],
    "total_sizes": [1024**3],
}


def convert_bytes(num_bytes: int) -> str:
    if num_bytes == 0:
        return "0B"
    size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
    index = int(math.floor(math.log(num_bytes, 1024)))
    power = math.pow(1024, index)
    size = round(num_bytes / power, 2)
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


def main():
    cleanup()
    configs = build_config()
    for label, config in configs.items():
        print(f"Running {label} ...")
        run_once(config, label)


if __name__ == "__main__":
    main()
