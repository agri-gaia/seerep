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

SEEREP_CFG_PATH = "tests/seerep.cfg"
DATA_FOLDER_CONFIG_KEY = "data-folder"
LOG_FOLDER_CONFIG_KEY = "log-path"
PORT_CONFIG_KEY = "port"

FB_GROUP_LIST_ENDS_WITH = ["IsNone", "Length"]
FB_FILTEROUT_STARTS_WITH = ["__"]
FB_FILTEROUT_ENDS_WITH = ["AsNumpy"]
FB_FILTEROUT_IS = [
    "_tab",
    "Init",
    "GetRootAs",
    "GetRootAsCameraIntrinsics",
    "GetRootAsCameraIntrinsicsQuery",
    "capitalize",
    "center",
    "count",
    "decode",
    "endswith",
    "expandtabs",
    "find",
    "fromhex",
    "hex",
    "index",
    "isalnum",
    "isalpha",
    "isascii",
    "isdigit",
    "islower",
    "isspace",
    "istitle",
    "isupper",
    "join",
    "ljust",
    "lower",
    "lstrip",
    "maketrans",
    "partition",
    "replace",
    "rfind",
    "rindex",
    "rjust",
    "rpartition",
    "rsplit",
    "rstrip",
    "split",
    "splitlines",
    "startswith",
    "strip",
    "swapcase",
    "title",
    "translate",
    "upper",
    "zfill",
]

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
        raise ValueError(
            'wrong formatting of tests/seerep.cfg. Port option "port" is missing. (default: port = 9095)'
        )

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
