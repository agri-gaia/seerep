import json
import os
import subprocess as sp
import tempfile as tf
from pathlib import Path
from typing import Dict, Final

SCHEMA_FOLDER: Final[str] = "seerep_msgs/fbs/"


<<<<<<< HEAD

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

    # count found endings and check if all endings are found
    for func in func_list:
        inner_tuple = []
        for ending in endings:
            if func + ending not in func_list:
                break
            else:
                inner_tuple.append(func + ending)
        else:
            dict_tuples[func] = inner_tuple
    return dict_tuples


def unpack_union_type(
    enclosing_table_obj,
    union_type_mapping: Dict[Type, Tuple[str, List[Tuple[int, Type]]]],
    snake_case=False,
    remappings: Dict[str, str] = {},
) -> Dict:
    union_field, field_nums_types = union_type_mapping[type(enclosing_table_obj)]

    # extract value type
    union_type = getattr(enclosing_table_obj, f"{union_field}Type")()
    union_obj = getattr(enclosing_table_obj, union_field)()

    for field_num, field_type in field_nums_types:
        if union_type == field_num:
            union_val = field_type()
            union_val.Init(union_obj.Bytes, union_obj.Pos)

            union_val = union_val.Data()

            if isinstance(union_val, bytes):
                try:
                    union_val = union_val.decode()
                except UnicodeDecodeError:
                    pass

            if isinstance(union_val, (int, float, str, bool)):
                return union_val

            return fb_obj_to_dict(union_val, snake_case, remappings, union_type_mapping)
    else:
        # ruff: noqa: PLW0120
        raise ValueError(f"union type {union_type} not in {field_nums_types} of table {type(enclosing_table_obj)}")


# todo: improve interface and rework functionality for more robustness
def fb_obj_to_dict(
    obj,
    snake_case=False,
    remappings: Dict[str, str] = {},
    union_type_mapping: Dict[Type, Tuple[str, List[Tuple[int, Type]]]] = {},
) -> Dict:
=======
def fb_flatc_dict(fb_obj: bytearray, schema_file_name: str) -> Dict:
>>>>>>> 2fdd6c52... cleanup of initial working fb_to_dict.py
    """
    Converts a binary flatbuffers object to a python dictionary using it's IDL file.
    This implementation uses temporary files in /tmp for conversion.

    Args:
<<<<<<< HEAD
        obj: The flatbuffer object to convert.
        snake_case: If the resulting dictionary keys (flatbuffer field names) should be converted to snake case.
        remappings: A dictionary of remappings for the resulting flatbuffer field names.
        union_type_mapping:
            union_type_mapping{ t: (field: str, [(num_type1, t_type1), (num_type2, t_type2), ...]) }
            where t is the parent table of the union (the table in which the union is a field of).
            The field is the name of the field containing the union.
            num_type1, num_type2, ... are the possible numbers identifying the type of the union.
            t_type1, t_type2, ... are the possible types of the union,
              which have to correpondend to num_type1, num_type2, ...
            For example:
                union_type_mapping: Dict[Type, Tuple[str, List[Tuple[int, Type]]]] = {
                    UnionMapEntry.UnionMapEntry: (
                        "Value",
                        [
                            (Datatypes.Datatypes().Boolean, Boolean.Boolean),
                            (Datatypes.Datatypes().Integer, Integer.Integer),
                            (Datatypes.Datatypes().Double, Double.Double),
                            (Datatypes.Datatypes().String, String.String),
                        ],
                    )
                }
    """
    # get all non dunder functions of the object
    funcs = [func for func in dir(obj) if callable(getattr(obj, func)) and not func.startswith("__")]

    # filter out the functions that are not needed
    funcs = [func for func in funcs if not any(func.startswith(x) for x in FB_FILTEROUT_STARTS_WITH)]
    funcs = [func for func in funcs if not any(func.endswith(x) for x in FB_FILTEROUT_ENDS_WITH)]
    # ruff: noqa: PLR1714
    funcs = [func for func in funcs if not any(func == x for x in FB_FILTEROUT_IS)]
    funcs = [func for func in funcs if not (func == "GetRootAs" or func == f"GetRootAs{type(obj).__name__}")]

    # group arrays and array functions together, so that arrays can be handled separately
    # e.g. (Distortion, DistortionIsNone, DistortionLength)
    farr_dict: Dict[str, Tuple] = get_func_lists_wparams(funcs, *FB_GROUP_LIST_ENDS_WITH)

    # filter funcs with endings out which are already in f_list
    funcs = [
        func
        for func in funcs
        if not any(func.endswith(x) if func.split(x)[0] in farr_dict else False for x in FB_GROUP_LIST_ENDS_WITH)
    ]
    funcs = [func for func in funcs if func not in farr_dict]

    res_dict = {}
    # get function results, if the result is a complex flatbuffer object, recursively call this function
    # ruff: noqa: PLW2901
    for func in funcs:
        if type(obj) in union_type_mapping and func == union_type_mapping[type(obj)][0]:
            res_dict[func] = unpack_union_type(obj, union_type_mapping, snake_case, remappings)
            continue
=======
        fb_obj: The bytearray object as returned by builder.Output().
        schema_file_name: The filename of the fb schema file.
>>>>>>> 2fdd6c52... cleanup of initial working fb_to_dict.py

    Returns:
        A python dictionary containing the objects attribute information.
    """
    schema_path = Path(SCHEMA_FOLDER + schema_file_name).absolute()

    with tf.NamedTemporaryFile(delete=False) as tmp_f:
        tmp_f.write(fb_obj)
        temp_fname = tmp_f.name
        flatc_proc = sp.Popen(
            ["flatc", "--json", "--raw-binary", "--strict-json", schema_path, "--", temp_fname], cwd="/tmp"
        )

    flatc_proc.wait()

    temp_json = temp_fname + ".json"
    with open(temp_json, "r") as tmp_f:
        json_dict = json.loads(tmp_f.read())

    try:
        os.remove(temp_json)
        os.remove(temp_fname)
    except FileNotFoundError:
        pass

    return json_dict
