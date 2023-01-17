#!/usr/bin/env python3
""" The SEEREP transfer CLI.

The CLI provides a simple interface to transfer and index HDF5 files from a robot to a running SEEREP server.
The program uses rsync to transfer the files and the SEEREP grpc API to notify the server about the new data.

Python 3.8 or higher is required and the dependencies are listed in the requirements.txt file.
"""

import ipaddress
import os
import re
import subprocess
from pathlib import Path
from shutil import which
from typing import Final, Optional

import flatbuffers
import grpc
import h5py
import typer
from rich.console import Console

# From TestPyPI https://test.pypi.org/project/seerep-grpc/
from seerep.fb import Empty, ProjectInfos
from seerep.fb import meta_operations_grpc_fb as metaOperations

DEFAULT_SERVER: Final[str] = "127.0.0.1"
DEFAULT_PORT: Final[int] = 9090
DEFAULT_DIR: Final[Path] = Path(os.getcwd())
DEFAULT_DATA_FOLDER: Final[Path] = Path("/seerep/seerep-data/")

app = typer.Typer()
console = Console()


def is_file(path: Path) -> Path:
    if path and not path.is_file():
        raise typer.BadParameter(f"Path '{path}' is not a file")
    if path and not is_hdf5(path):
        raise typer.BadParameter(f"File '{path.name}' is not a HDF5 file")
    return path


def is_dir(path: Path) -> Path:
    if path and not path.is_dir():
        raise typer.BadParameter(f"Path '{path}' is not a directory")
    return path


def valid_port(port: int) -> int:
    if port < 1024 or port > 65535:
        raise typer.BadParameter(f"Port '{port}' is not a valid port number")
    return port


def valid_ip(ip_address: str) -> str:
    try:
        ipaddress.ip_address(ip_address)
    except ValueError:
        raise typer.BadParameter(f"IP address '{ip_address}' is not a valid IP address")
    return ip_address


def is_hdf5(file_path: Path) -> Path:
    if not h5py.is_hdf5(file_path):
        raise typer.BadParameter(f"File '{file_path.name}' is not a HDF5 file")
    return file_path


def valid_user(user: str) -> str:
    if not bool(re.search("^[a-z_]([a-z0-9_-]{0,31}|[a-z0-9_-]{0,30}\$)$", user)):
        raise typer.BadParameter(f"Username '{user}' is not valid")
    return user


def server_available(channel: grpc.Channel, timeout: int = 5) -> bool:
    """Check if the specifies SEEREP server is available."""
    try:
        grpc.channel_ready_future(channel).result(timeout)
        return True
    except grpc.FutureTimeoutError:
        return False


def rsync_installed() -> bool:
    """Check if rsync is installed."""
    return which("rsync") is not None


def build_empty_msg() -> bytes:
    """Build an empty flatbuffers message."""
    builder = flatbuffers.Builder(1024)
    Empty.Start(builder)
    empty_msg = Empty.End(builder)
    builder.Finish(empty_msg)
    buf = builder.Output()
    return buf


def get_projects_from_server(channel: grpc.Channel) -> ProjectInfos.ProjectInfos:
    """Get all projects from the SEEREP server."""
    stub = metaOperations.MetaOperationsStub(channel)
    responseBuf = stub.GetProjects(bytes(build_empty_msg()))
    return ProjectInfos.ProjectInfos.GetRootAs(responseBuf)


def load_new_project(channel: grpc.Channel) -> ProjectInfos.ProjectInfos:
    """Load a new project on the SEEREP server."""
    stub = metaOperations.MetaOperationsStub(channel)
    responseBuf = stub.LoadProjects(bytes(build_empty_msg()))
    return ProjectInfos.ProjectInfos.GetRootAs(responseBuf)


def execute_command(cmd: str) -> None:
    """Execute a command in a subprocess and get the output lines back"""
    process = subprocess.Popen(
        cmd,
        shell=True,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        universal_newlines=True,
    )
    for stdout_line in iter(process.stdout.readline, ""):
        yield stdout_line
    process.stdout.close()
    return_code = process.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)


# TODO: Think about what happens, if the server goes down between here and the function calls below
def send_file(local_file: Path, username: str, server: str, port: int, data_folder_path: Path) -> None:
    """Send a single HDF5 file to a SEEREP server using rsync and index it."""

    channel = grpc.insecure_channel(server + ":" + str(port))

    local_uuid = local_file.name.split(".")[0]
    projects_on_server = get_projects_from_server(channel)

    for i in range(projects_on_server.ProjectsLength()):
        if projects_on_server.Projects(i).Uuid().decode("utf-8") == local_uuid:
            console.print(f"File '{local_file.name}' already present on SEEREP '{server}:{port}'")
            return

    # NOTE: currently limited to 1MB/s
    # Uses zlib for compression (algorithm used in gzip)
    cmd = (
        f"{which('rsync')} -z --progress --bwlimit=100000 {str(local_file)} {username}@{server}:{str(data_folder_path)}"
    )

    print(f"\nTransfering file ...")
    for output in execute_command(cmd):
        print("\r " + output.strip(), end="\r", flush=True)

    newly_indexed_projects = load_new_project(channel)

    if newly_indexed_projects.ProjectsLength() == 0:
        console.print(f"[red] File not read by the server.")
        return

    responseFileName = newly_indexed_projects.Projects(0).Uuid().decode("utf-8")
    if responseFileName == local_file.name.split(".")[0]:
        console.print(
            f"File '{local_file.name}' successfully transferred to SEEREP '{server}:{port}' and loaded into the index"
        )


@app.command()
def push(
    server: str = typer.Option(DEFAULT_SERVER, help="IP to a running SEEREP server", callback=valid_ip),
    user: str = typer.Option(..., help="Username to use for the SSH connection", callback=valid_user),
    port: int = typer.Option(DEFAULT_PORT, help="Port of the SEEREP server", callback=valid_port),
    file: Optional[Path] = typer.Option(None, help="Path to HDF5 file to send", callback=is_file),
    dir: Optional[Path] = typer.Option(
        DEFAULT_DIR,
        help="Path to directory with HDF5 files to send. The current dir is used by default.",
        callback=is_dir,
    ),
    # TODO: fix dir check for remote path
    data_folder: Optional[Path] = typer.Option(
        DEFAULT_DATA_FOLDER, help="Full path to the storage folder on the SEEREP server.", callback=is_dir
    ),
):
    """Send HDF5 file(s) to a running SEEREP server and load them into the index."""

    if not rsync_installed():
        console.print("[red] rsync is not installed, please install rsync using apt")
        raise typer.Exit()

    if data_folder != DEFAULT_DATA_FOLDER and not data_folder.is_absolute():
        console.print("[red] Please specify the full path to the data folder on the SEEREP server")
        raise typer.Exit()

    if not server_available(grpc.insecure_channel(f"{server}:{str(port)}")):
        console.print(f"[red] SEEREP server '{server}:{str(port)}' is not available")
        raise typer.Exit()

    if file and dir != DEFAULT_DIR:
        console.print("[orange] Please specify either a file or a directory, not both")
        raise typer.Exit()

    # TODO add support for directories
    try:
        if file:
            send_file(file, user, server, port, data_folder)
    except Exception as e:
        console.print(f"[red] Error: {e}")
        raise typer.Exit()


if __name__ == "__main__":
    app()
