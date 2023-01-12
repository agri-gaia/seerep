#!/usr/bin/env python3

# Debugging
import code
import ipaddress
import os
import re
import subprocess
from pathlib import Path
from typing import Final, Optional

import flatbuffers
import grpc
import h5py
import typer
from rich.console import Console

# From TestPyPI https://test.pypi.org/project/seerep-grpc/
from seerep.fb import Empty
from seerep.fb import meta_operations_grpc_fb as metaOperations

DEFAULT_SERVER: Final[str] = "127.0.0.1"
DEFAULT_PORT: Final[int] = 9090
DEFAULT_DIR: Final[Path] = Path(os.getcwd())
DEFAULT_DATA_FOLDER: Final[Path] = Path("/seerep/seerep-data/")

app = typer.Typer()
console = Console()

# General TODOS
# TODO make actions on server idempotent


def is_file(path: Path):
    if path and not path.is_file():
        raise typer.BadParameter(f"Path '{path}' is not a file")
    if path and not is_hdf5(path):
        raise typer.BadParameter(f"File '{path.name}' is not a HDF5 file")
    return path


def is_dir(path: Path):
    if path and not path.is_dir():
        raise typer.BadParameter(f"Path '{path}' is not a directory")
    return path


def valid_port(port: int):
    if port < 1024 or port > 65535:
        raise typer.BadParameter(f"Port '{port}' is not a valid port number")
    return port


def valid_ip(ip_address: str):
    try:
        ipaddress.ip_address(ip_address)
    except ValueError:
        raise typer.BadParameter(f"IP address '{ip_address}' is not a valid IP address")
    return ip_address


def is_hdf5(file_path: Path):
    if not h5py.is_hdf5(file_path):
        raise typer.BadParameter(f"File '{file_path.name}' is not a HDF5 file")
    return file_path


def valid_user(user: str):
    if not bool(re.search("^[a-z_]([a-z0-9_-]{0,31}|[a-z0-9_-]{0,30}\$)$", user)):
        raise typer.BadParameter(f"Username '{user}' is not valid")
    return user


def grpc_server_available(channel: grpc.Channel, timeout: int = 5) -> bool:
    """Check if a gRPC server is available."""
    try:
        grpc.channel_ready_future(channel).result(timeout)
        return True
    except grpc.FutureTimeoutError:
        return False


def send_file(local_file: Path, username: str, server: str, port: int, data_folder_path: Path) -> bool:
    """Send a single HDF5 file to a SEEREP server using rsync."""

    # TODO: Think about what happens, if the server goes down between here and the function calls below
    channel = grpc.insecure_channel(server + ":" + str(port))
    if not grpc_server_available(channel):
        console.print(f"[bold red] No running SEEREP server on {server}:{str(port)}")
        return False

    # TODO: progress bar with current status of the transfer possible?
    subprocess.call(["rsync", "-a", str(local_file), username + "@" + server + ":" + str(data_folder_path)])

    stub = metaOperations.MetaOperationsStub(channel)
    builder = flatbuffers.Builder(1024)

    Empty.Start(builder)
    empty_msg = Empty.End(builder)
    builder.Finish(empty_msg)
    buf = builder.Output()

    # TODO: return how many files were read in correctly
    response = stub.LoadProjects(bytes(buf))
    responseBuf = Empty.Empty.GetRootAsEmpty(response)

    if responseBuf:
        console.print(f"[bold green] File '{local_file.name}' sent to server '{server}'")


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
    data_folder: Optional[Path] = typer.Option(
        DEFAULT_DATA_FOLDER, help="Path to the storage folder on the SEEREP server.", callback=is_dir
    ),
):
    """Send HDF5 file(s) to a SEEREP server and load them into the index."""
    print("server: " + server)
    if file and dir != DEFAULT_DIR:
        console.print("Please specify either a file or a directory, not both")
        raise typer.Exit()
    if file:
        send_file(file, user, server, port, data_folder)


if __name__ == "__main__":
    app()
