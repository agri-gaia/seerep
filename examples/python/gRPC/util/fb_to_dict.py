# this module needs a revamp and tests for all possible flatbuffer type scenarios
# the current implementation is dependent on hardcoded assumptions
import re
from typing import Dict, Iterable, List, Tuple, Type

FB_GROUP_LIST_ENDS_WITH = ["IsNone", "Length"]
FB_FILTEROUT_STARTS_WITH = ["__", "GetRootAs"]
FB_FILTEROUT_ENDS_WITH = ["AsNumpy", "Type"]
FB_FILTEROUT_IS = [
    "_tab",
    "Init",
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


UPPER_CASE_LETTERS = r"([A-Z])"
RPLC_UPPER_CASE_LETTERS = r"_\1"


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
        inner_tuple = tuple()
        for ending in endings:
            if not func + ending in func_list:
                break
            else:
                inner_tuple += tuple([func + ending])
        else:
            dict_tuples[func] = inner_tuple
    return dict_tuples


def unpack_union_type(
    enclosing_table_obj,
    union_type_mapping: Dict[Type, Tuple[str, List[Tuple[int, Type]]]],
    to_snake_case=False,
    remappings: Dict[str, str] = dict(),
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

            return fb_obj_to_dict(
                union_val, to_snake_case, remappings, union_type_mapping
            )
    else:
        raise ValueError(
            f"union type {union_type} not in {field_nums_types} of table {type(enclosing_table_obj)}"
        )


# union_type_mapping{ t: (field, [(num_type1, t_type1), (num_type2, t_type2), ...]) }
# where t is the table enclosing the union
# field is the name of the field containing the union
# t_type1, t_type2, ... are the possible types of the union
# todo: rework remappings field and functionality


def fb_obj_to_dict(
    obj,
    to_snake_case=False,
    remappings: Dict[str, str] = dict(),
    union_type_mapping: Dict[Type, Tuple[str, List[Tuple[int, Type]]]] = dict(),
) -> Dict:
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
    farr_dict: Dict[str, Tuple] = get_func_lists_wparams(
        funcs, *FB_GROUP_LIST_ENDS_WITH
    )

    # filter funcs with endings out which are already in f_list
    funcs = [
        func
        for func in funcs
        if not any(
            [
                func.endswith(x) if func.split(x)[0] in farr_dict else False
                for x in FB_GROUP_LIST_ENDS_WITH
            ]
        )
    ]
    funcs = [func for func in funcs if not func in farr_dict]

    res_dict = {}
    # get function results, if the result is a complex flatbuffer object, recursively call this function
    for func in funcs:
        if type(obj) in union_type_mapping and func == union_type_mapping[type(obj)][0]:
            res_dict[func] = unpack_union_type(
                obj, union_type_mapping, to_snake_case, remappings
            )
            continue

        res = getattr(obj, func)()

        if isinstance(res, bytes):
            res = res.decode()

        if to_snake_case:
            if res == 0:
                continue

            func = (
                func[0] + re.sub(UPPER_CASE_LETTERS, RPLC_UPPER_CASE_LETTERS, func[1:])
            ).lower()

            if func in remappings:
                func = remappings[func]

        # check if resulting type is a complex type by checking if the resulting type is a primitive type
        if isinstance(res, (int, float, str, bool)):
            res_dict[func] = res
        else:
            res_dict[func] = fb_obj_to_dict(
                res, to_snake_case, remappings, union_type_mapping
            )

    # do the same for arrays
    for f_key in farr_dict:
        if getattr(obj, farr_dict[f_key][0])():
            if to_snake_case:
                f_key = (
                    f_key[0]
                    + re.sub(UPPER_CASE_LETTERS, RPLC_UPPER_CASE_LETTERS, f_key[1:])
                ).lower()
                if f_key in remappings:
                    f_key = remappings[f_key]

            res_dict[f_key] = []
            continue

        res_lst = []
        for i in range(getattr(obj, farr_dict[f_key][1])()):
            res = getattr(obj, f_key)(i)

            if isinstance(res, bytes):
                res = res.decode()

            if to_snake_case and res == 0:
                continue

            if isinstance(res, (int, float, str, bool)):
                res_lst.append(res)
            else:
                res_lst.append(
                    fb_obj_to_dict(res, to_snake_case, remappings, union_type_mapping)
                )

        # find first occurence of upper case letter replace it with underscore and lower case letter
        if to_snake_case:
            f_key = (
                f_key[0]
                + re.sub(UPPER_CASE_LETTERS, RPLC_UPPER_CASE_LETTERS, f_key[1:])
            ).lower()
            if f_key in remappings:
                f_key = remappings[f_key]

        res_dict[f_key] = res_lst

    return res_dict
