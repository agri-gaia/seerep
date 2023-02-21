#!/usr/bin/env python3

# NOTE: run with rosrun seerep_performance_tests run.py

from pathlib import Path
import subprocess
import math

# a file with bytes to use as a message payload
OUTPUT_DIR = Path("/home/pbrstudent/Documents/seerep-data")
MESSAGE_DATA_PATH = Path("/home/pbrstudent/Documents/rosbags/iros/center-rgb8-only.mcap")

CONFIG = {
    # both sizes must be in bytes
    "message_sizes": [1000**2 * 100],
    "total_sizes": [1000**3, 1000**3 * 3],
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

def executable_path() -> Path:
    try:
        output = subprocess.run(
            ["catkin_find", "seerep_performance_tests",  "seerep_performance_tests"],
            check=True, 
            stdout=subprocess.PIPE
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError("Could not find path to executable.")
    return output.stdout.decode("utf-8").strip()

def run_once(config: dict, label: str) -> None:
    try:
        output = subprocess.run(
            [executable_path(), 
            str(MESSAGE_DATA_PATH),
            label,
            OUTPUT_DIR,
            str(config["message_size"]),
            str(config["total_size"])],
            check=True,
            stdout=subprocess.PIPE,
        )
        print(output.stdout.decode("utf-8").strip())
    except subprocess.CalledProcessError as e:
        raise RuntimeError("Error running executable.")
    

def main():
    configs = build_config()
    for label, config in configs.items():
        print(f"Running {label} ...")
        run_once(config, label)

if __name__ == "__main__":
    main()