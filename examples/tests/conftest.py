import os
from typing import Dict, Iterable, Tuple

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


def get_func_lists_wparams(func_list, *endings) -> Dict[str, Tuple[bool, int]]:
    """
    Returns a list of tuples of functions that have versions of them with certain endings.
    The first element of the tuple is the function name, the following elements are the results of the extra functions.
    based on the function existence, the corresponding function is assumed to be a array.
    the contained tuple are of the form (array, array_is_none, array_length)
    """
    if not isinstance(endings, Iterable):
        raise ValueError("endings must be iterable")

    # this will have the form (array, array_is_none, array_length)
    dict_tuples = {}
    # collect all functions that end with certain strings
    for func in func_list:
        inner_tuple = tuple()
        for ending in endings:
            if not func + ending in func_list:
                break
            else:
                inner_tuple += tuple([func + ending])
        else:
            dict_tuples[func] = inner_tuple
    return dict_tuples


def fb_obj_to_dict(obj) -> Dict:
    """
    Converts a flatbuffer object to a dictionary.
    """
    # get all non dunder functions of the object
    funcs = [
        func
        for func in dir(obj)
        if callable(getattr(obj, func)) and not func.startswith("__")
    ]

    # filter out the functions that are not needed
    funcs = [
        func
        for func in funcs
        if not any([func.startswith(x) for x in FB_FILTEROUT_STARTS_WITH])
    ]
    funcs = [
        func
        for func in funcs
        if not any([func.endswith(x) for x in FB_FILTEROUT_ENDS_WITH])
    ]
    funcs = [func for func in funcs if not any([func == x for x in FB_FILTEROUT_IS])]
    funcs = [
        func
        for func in funcs
        if not (func == "GetRootAs" or func == f"GetRootAs{type(obj).__name__}")
    ]

    # group arrays and array functions together, so that arrays can be handled separately
    # e.g. (Distortion, DistortionIsNone, DistortionLength)
    f_dict: Dict[str, Tuple] = get_func_lists_wparams(funcs, *FB_GROUP_LIST_ENDS_WITH)

    # filter funcs with endings out which are already in f_list
    funcs = [
        func
        for func in funcs
        if not any(
            [
                func.endswith(x) if func.split(x)[0] in f_dict else False
                for x in FB_GROUP_LIST_ENDS_WITH
            ]
        )
    ]
    funcs = [func for func in funcs if not func in f_dict]

    res_dict = {}
    # get function results, if the result is a complex flatbuffer object, recursively call this function
    for func in funcs:
        res = getattr(obj, func)()
        if isinstance(res, bytes):
            res = res.decode()
        # check if resulting type is a complex type by checking if the resulting type is a primitive type
        if isinstance(res, (int, float, str, bool)):
            res_dict[func] = res
        else:
            res_dict[func] = fb_obj_to_dict(res)

    # do the same for arrays
    for f_key in f_dict:
        if getattr(obj, f_dict[f_key][0])():
            res_dict[f_key] = []
            continue
        res_lst = []
        for i in range(getattr(obj, f_dict[f_key][1])()):
            res = getattr(obj, f_key)(i)
            if isinstance(res, bytes):
                res = res.decode()
            if isinstance(res, (int, float, str, bool)):
                res_lst.append(res)
            else:
                res_lst.append(fb_obj_to_dict(res))
        res_dict[f_key] = res_lst

    return res_dict


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
