#!/usr/bin/env python3

import math
import os
import re
import subprocess
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
from typing import Tuple, Union

import matplotlib.pyplot as plt
import pandas as pd

plt.rcParams.update(
    {
        "text.usetex": True,
        "font.family": "serif",
        "axes.labelsize": 11,
        "axes.titlesize": 10,
        "font.size": 11,
        "legend.fontsize": 6,
        "legend.title_fontsize": 7,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
    }
)

# a file with bytes to use as a message payload
OUTPUT_DIR = Path("/seerep/seerep-data/benchmarks")
MESSAGE_PAYLOAD = Path("/seerep/seerep-data/lero/2022-08-24-12-21-43_0.mcap")

CONFIG = {
    "num_runs": 50,
    "message_sizes": [
        1024 * 10,
        1024 * 100,
        1024 * 200,  # Somewhere here they should be the same
        1024 * 600,  # Lero Depth Map
        1024**2 * 1,
        1024**2 * 4,  # Full HD 1 Channel
        1024**2 * 6,  # Full HD 3 Channels
    ],
    "total_sizes": [1024**2 * 250],
}


def bytes_to_string(num_bytes: int) -> str:
    """Convert a number of bytes to a human readable string."""
    if num_bytes == 0:
        return "0B"
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    unit_index = int(math.floor(math.log(num_bytes, 1024)))
    bytes_in_unit = int(round(num_bytes / math.pow(1024, unit_index)))
    return f"{bytes_in_unit}{units[unit_index]}"


def string_to_bytes(byte_str: str) -> int:
    UNIT_MAP = {
        "B": 1,
        "KiB": 1024,
        "MiB": 1024**2,
        "GiB": 1024**3,
        "TiB": 1024**4,
    }
    split_result = re.split(r'(\d+\.\d+|\d+)', byte_str)
    return int(split_result[1]) * UNIT_MAP[split_result[2]]


def build_config(config: dict) -> dict:
    """Build all combinations for the given configuration."""
    configs = {}
    for message_size in config["message_sizes"]:
        for total_size in config["total_sizes"]:
            label = f"{bytes_to_string(message_size)}-{bytes_to_string(total_size)}"
            configs[label] = {
                "message_size": message_size,
                "total_size": total_size,
            }
    return configs


def benchmark_exc_path() -> Path:
    """Get the path to the benchmark executable"""
    try:
        subprocess_output = subprocess.run(
            ["catkin_find", "seerep_performance_tests", "seerep_performance_tests"], check=True, stdout=subprocess.PIPE
        )
    except subprocess.CalledProcessError as e:
        print(f"Subprocess error while searching for benchmark executable: {e}")
        exit(1)
    return Path(subprocess_output.stdout.decode("utf-8").strip())


def remove_files(dir: Path, extension: str) -> None:
    """Remove all files with a given extension in the directory."""
    for file in Path.glob(dir, "*" + extension):
        try:
            os.remove(file)
        except OSError as e:
            print(f"Error while deleting {file} because: {e.strerror}")


def setup(output_dir: Path, dirs: list = ["mcap", "hdf5"], clean_csv: bool = True) -> None:
    """Cleanup before running the benchmark."""
    for dir in dirs:
        path = Path(f"{output_dir}/{dir}")
        if not path.exists():
            path.mkdir()
        else:
            remove_files(path, f".{dir}")
    if clean_csv:
        remove_files(output_dir, ".csv")


def run_config(config: dict, label: str, payload_path: Path, output_dir: Path, subprocess_output: bool = False) -> None:
    """Run the configuration of the performance test once."""

    benchmark_path = benchmark_exc_path()
    if not benchmark_path != Path("."):
        print("Could not find benchmark executable, run catkin build and source the workspace!")
        exit(1)

    try:
        output = subprocess.run(
            [
                benchmark_path,
                str(payload_path),
                label,
                str(output_dir),
                str(config["message_size"]),
                str(config["total_size"]),
            ],
            check=True,
            stdout=subprocess.PIPE,
        )
    except subprocess.CalledProcessError as e:
        print(f"Subprocess error while running benchmark: {e}")
        exit(1)
    if subprocess_output:
        print(f"{output.stdout.decode('utf-8')}")


def read_and_concat_csvs(csv_path: Path) -> Union[pd.DataFrame, None]:
    """Read all csv files in a given directory and return a united dataframe."""
    dfs = [pd.read_csv(csv_file).assign(label=csv_file.stem) for csv_file in Path.glob(csv_path, "*.csv")]
    return pd.concat(dfs, ignore_index=True) if dfs else None


def process_data(df: pd.DataFrame) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """Process the read csv data for visualization."""

    # convert to seconds
    df["write_ns"] = df["write_ns"].apply(lambda x: x / 1e9)
    df.rename(columns={"write_ns": "write_s"}, inplace=True)

    # convert to GB
    df["written_bytes"] = df["written_bytes"].apply(lambda x: x / 1024**3)
    df.rename(columns={"written_bytes": "written_gb"}, inplace=True)

    # add extra columns for the msg size and file_format
    df["msg_size"] = df["label"].apply(lambda x: x.split("-")[1])
    df["file_format"] = df["label"].apply(lambda x: x.split("-")[0].upper())

    df["gb/s"] = df[["written_gb", "write_s"]].apply(lambda x: x.written_gb / x.write_s, axis=1)

    # get the mean and std for each configuration
    df = df.groupby(["label", "msg_size", "file_format"], as_index=False)["gb/s"].agg(mean="mean", std="std")

    # don't need this column anymore
    df.drop(columns=["label"], inplace=True)

    # change to the required shape for bar plots
    mean_df = df.pivot(index="msg_size", columns="file_format", values="mean")
    std_df = df.pivot(index="msg_size", columns="file_format", values="std")

    # sort by ascending msg sie
    mean_df.sort_values(by="msg_size", inplace=True, key=lambda x: x.apply(string_to_bytes))
    std_df.sort_values(by="msg_size", inplace=True, key=lambda x: x.apply(string_to_bytes))

    return (mean_df, std_df)


def set_size(width: float, fraction: float = 1) -> Tuple[float, float]:
    """Set figure dimensions to avoid scaling in LaTeX."""

    # Width of figure (in pts)
    fig_width_pt = width * fraction

    # Convert from pt to inches
    inches_per_pt = 1 / 72.27

    # Golden ratio to set aesthetic figure height
    # https://disq.us/p/2940ij3
    golden_ratio = (5**0.5 - 1) / 2

    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio

    fig_dim = (fig_width_in, fig_height_in)

    return fig_dim


def plot_data(mean_df: pd.DataFrame, std_df: pd.DataFrame, save_img: bool = True) -> None:
    """Plot or save the processed data in a bar chart."""
    # TODO: automatically generate the title
    title = f"250 MiB Total Data For Each Message Size \
        \n MCAP: No Compression - Default Chunking  \
        \n HDF5: No Compression - No Chunking"

    ax = mean_df.plot(
        kind="bar",
        width=0.4,
        yerr=std_df,
        color=["#a6a6a6", "#548235"],
        rot=45,
        capsize=2,
        edgecolor='white',
        linewidth=1,
        figsize=set_size(245.72, 1.3),
    )

    # plt.title(title, pad=15)
    plt.xlabel(
        "Message Size",
        labelpad=10,
    )
    plt.ylabel(
        "Troughput in GiB/s",
    )
    plt.xticks()
    plt.yticks()

    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(axis="y", linestyle="dashed", linewidth=0.5, alpha=0.65)
    ax.set_axisbelow(True)
    ax.legend(title="File Format", loc="upper left")
    plt.tight_layout()

    if save_img:
        plt.savefig(
            Path(f"{OUTPUT_DIR}/benchmark_run_{datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}.pdf"),
            format="pdf",
            bbox_inches="tight",
        )
    else:
        plt.show()


def main():

    parser = ArgumentParser()
    parser.add_argument("--only-plot", action="store_true", help="Only plot the data from the csv files.")
    parser.add_argument("--save-img", action="store_true", help="Save the plot as an image.")
    args = parser.parse_args()

    if not args.only_plot:
        config_combinations = build_config(CONFIG)
        setup(OUTPUT_DIR)
        for label, config in config_combinations.items():
            print(f"Running {label} ...")
            for run in range(CONFIG["num_runs"]):
                print(f"Run {run + 1} ...")
                run_config(config, label, MESSAGE_PAYLOAD, OUTPUT_DIR)
                # remove hdf5 and mcap files after each run
                setup(OUTPUT_DIR, clean_csv=False)

    print("Reading csv files ...")
    csv_df = read_and_concat_csvs(OUTPUT_DIR)
    if csv_df is None:
        print("No csv files found, exiting ...")
        exit(1)
    print("Processing data ...")
    mean_df, std_df = process_data(csv_df)
    print("Generating plot ...")
    plot_data(mean_df, std_df, save_img=True)


if __name__ == "__main__":
    main()
