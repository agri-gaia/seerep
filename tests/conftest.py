import shutil
import subprocess
import time
from pathlib import Path
from typing import Generator, Tuple

import flatbuffers
import pytest
import setup_util
from grpc import Channel
from gRPC.meta import gRPC_pb_createProject as proj_creation
from seerep.util import fb_helper
from seerep.util.common import get_gRPC_channel

SERVER_TEST_CONFIG_PATH = Path(__file__).resolve().parent / "seerep.cfg"
SERVER_EXECUTABLE_NAME = "seerep_server"


@pytest.fixture(scope="session")
def server_config() -> Generator[dict, None, None]:
    yield setup_util.get_server_config(SERVER_TEST_CONFIG_PATH)


# TODO: Compile server executable if not present? Make sure it has been build in
# Release mode?
@pytest.fixture(scope="session")
def grpc_channel(server_config: dict) -> Generator[Channel, None, None]:
    # find the server executable
    server_exe_path = shutil.which(SERVER_EXECUTABLE_NAME)
    if not server_exe_path:
        raise FileNotFoundError(
            f"Server executable '{SERVER_EXECUTABLE_NAME}' not found"
        )

    # remove any leftover files from previous tests
    setup_util.remove_hdf5_files(server_config["data-folder"])
    setup_util.remove_log_files(server_config["log-path"])

    # start server in the background
    server_process = subprocess.Popen(
        [server_exe_path, f"-c{SERVER_TEST_CONFIG_PATH}"],
        stdout=subprocess.DEVNULL,
    )

    if not server_process:
        raise RuntimeError("Failed to start SEEREP")

    # wait for the gRPC interface to be available
    while not setup_util.is_port_open(server_config["port"]):
        time.sleep(0.1)

    yield get_gRPC_channel(f"localhost:{server_config['port']}")

    server_process.terminate()


# NOTE: This fixture depends on working project creation and deletion
@pytest.fixture
def project_setup(grpc_channel) -> Generator[Tuple[str, str], None, None]:
    project_name, project_uuid = proj_creation.create_project(grpc_channel)

    yield project_name, project_uuid

    fb_helper.deleteProject(
        grpc_channel, flatbuffers.Builder(), project_name, project_uuid
    )
