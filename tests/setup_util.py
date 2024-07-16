import socket
from pathlib import Path


def get_server_config(config_path: str) -> dict:
    """
    Retrieves the server configuration from a given config file.

    Args:
        config_path (str): The path to the config file (.cfg)

    Returns:
        dict: A dictionary containing the server configuration.

    Raises:
        ValueError: If the config file is not found or if it is missing required
        settings.
    """
    REQUIRED_KEYS = ["data-folder", "log-path", "port"]
    config_path = Path(config_path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Config file not found at '{config_path}'")
    config = {}
    with open(config_path, "r") as f:
        lines = f.read().strip().split("\n")
        for line in lines:
            if line.startswith("#"):
                continue
            key, value = line.split("=")
            value = (
                value.split("#")[0].strip() if "#" in value else value.strip()
            )
            config[key.strip()] = int(value) if value.isdigit() else value
    for key in REQUIRED_KEYS:
        if key not in config:
            raise ValueError(
                f"Config file at '{config_path}' is missing required setting \
                '{key}'"
            )
    return config


def remove_files(directory: str, file_ending: str) -> None:
    """
    Removes files with the specified file ending in the given directory.

    Args:
        path (str): The path to the directory
        file_ending (str): The file ending of the files to remove
        (without the dot)

    Returns:
        None
    """
    if file_ending and "." in file_ending:
        raise ValueError(
            "Invalid file ending: File extensions must not contain a dot"
        )
    directory = Path(directory)
    if directory.is_dir():
        log_files = [
            f for f in directory.glob(f"*.{file_ending}") if f.is_file()
        ]
        for file in log_files:
            file.unlink()


def remove_hdf5_files(directory: str) -> None:
    """
    Removes HDF5 files in the given directory.

    Args:
        path (str): The path to the directory

    Returns:
        None
    """
    remove_files(directory, "h5")


def remove_log_files(directory: str) -> None:
    """
    Removes log files in the given directory.

    Args:
        path (str): The path to the directory

    Returns:
        None
    """
    remove_files(directory, "log")


def is_port_open(port: int, host: str = "localhost") -> bool:
    """
    Checks if a port is open on the given host.

    Args:
        port (int): The port to check
        host (str): The host to check the port on

    Returns:
        bool: True if the port is open, False otherwise
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        return sock.connect_ex((host, port)) == 0
