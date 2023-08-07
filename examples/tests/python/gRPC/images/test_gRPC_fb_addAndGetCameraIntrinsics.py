from collections.abc import Iterable
from typing import Dict, List, Tuple

import conftest
import flatbuffers
from gRPC.images import gRPC_fb_addCameraIntrinsics as add_ci
from gRPC.images import gRPC_fb_getCameraIntrinsics as get_ci
from seerep.fb import CameraIntrinsics


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
        if not any([func.startswith(x) for x in conftest.FB_FILTEROUT_STARTS_WITH])
    ]
    funcs = [
        func
        for func in funcs
        if not any([func.endswith(x) for x in conftest.FB_FILTEROUT_ENDS_WITH])
    ]
    funcs = [
        func for func in funcs if not any([func == x for x in conftest.FB_FILTEROUT_IS])
    ]
    funcs = [
        func
        for func in funcs
        if not (func == "GetRootAs" or func == f"GetRootAs{type(obj).__name__}")
    ]

    # group arrays and array functions together, so that arrays can be handled separately
    # e.g. (Distortion, DistortionIsNone, DistortionLength)
    f_dict: Dict[str, Tuple] = get_func_lists_wparams(
        funcs, *conftest.FB_GROUP_LIST_ENDS_WITH
    )

    # filter funcs with endings out which are already in f_list
    funcs = [
        func
        for func in funcs
        if not any(
            [
                func.endswith(x) if func.split(x)[0] in f_dict else False
                for x in conftest.FB_GROUP_LIST_ENDS_WITH
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


def test_addAndGetCameraIntrinsics(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    sent_ci = add_ci.add_camintrins(grpc_channel, proj_uuid)

    fb_obj_to_dict(sent_ci)
