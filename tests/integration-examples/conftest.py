import os
from typing import Tuple

import flatbuffers
import pytest
from gRPC.meta import gRPC_pb_createProject as proj_creation
from seerep.util import fb_helper
from seerep.util.common import get_gRPC_channel

# tests are only working with a seerep server listening on the port specified in ./seerep.cfg

# pytest starts relative to where it finds the first pyproject.toml
# in this case it is in the examples folder

SEEREP_CFG_PATH = "tests/integration-examples/seerep.cfg"
DATA_FOLDER_CONFIG_KEY = "data-folder"
LOG_FOLDER_CONFIG_KEY = "log-path"
PORT_CONFIG_KEY = "port"


# start the server once
@pytest.fixture(scope="session")
def retrieve_server_params(request):
    # todo: use a more robust method to read the config file
    # read relevant values from seerep.cfg
    with open(os.path.join(request.config.rootdir, SEEREP_CFG_PATH), "r") as f:
        lines = f.read().split("\n")

    kv_dict = {}
    for kv_wcomments in lines:
        kv_no_spaces = kv_wcomments.replace(" ", "")
        # get rid of comments
        kv_pairs = kv_wcomments.split("#")[0]
        # skip if there is no = in the relevant string
        if "=" not in kv_pairs:
            continue
        kv_tuple = kv_pairs.split("=", maxsplit=1)

        try:
            kv_dict[kv_tuple[0].strip()] = int(kv_tuple[1])
        except ValueError:
            kv_dict[kv_tuple[0].strip()] = kv_tuple[1]

    if PORT_CONFIG_KEY not in kv_dict or not isinstance(kv_dict[PORT_CONFIG_KEY], int):
        raise ValueError('wrong formatting of tests/seerep.cfg. Port option "port" is missing. (default: port = 9095)')

    yield kv_dict[PORT_CONFIG_KEY]


# provide a instance of the grpc_channel to the tests
@pytest.fixture(scope="session")
def grpc_channel(retrieve_server_params):
    channel = get_gRPC_channel(f"localhost:{retrieve_server_params}")
    yield channel


# this fixture depends on working project creation and deletion
@pytest.fixture
def project_setup(grpc_channel) -> Tuple[str, str]:
    # setup
    proj_name, proj_uuid = proj_creation.create_project(grpc_channel)
    yield proj_name, proj_uuid

    # teardown
    builder = flatbuffers.Builder()
    fb_helper.deleteProject(grpc_channel, builder, proj_name, proj_uuid)


# creates a project with geodetic coordinates
@pytest.fixture
def geo_project_setup(grpc_channel) -> Tuple[str, str]:
    # setup
    builder = flatbuffers.Builder()

    # should correspondend to topocentric coordinates:
    # X: 3876524.5192
    # Y: 548466.2095
    # Z: 5018349.9785
    proj_info = fb_helper.createProjectRaw(
        grpc_channel,
        builder,
        "testproject",
        "2",
        "EPSG::4326",
        93,
        52.2264,
        8.0530,
    )

    proj_name, proj_uuid = proj_creation.create_project(grpc_channel)
    yield proj_name, proj_uuid

    # teardown
    fb_helper.deleteProject(grpc_channel, builder, proj_name, proj_uuid)
