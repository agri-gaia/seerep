import os
import re
from copy import deepcopy
from typing import Any, Dict

import grpc


def get_gRPC_channel(target="local"):
    # set the max message size to 1GB (half the size of the server)
    options = [
        ("grpc.max_send_message_length", 1 * 1024 * 1024 * 1024),
        ("grpc.max_receive_message_length", 1 * 1024 * 1024 * 1024),
    ]

    if target == "prod" or target == "dev":
        if target == "prod":
            targetName = "seerep-prod"
            certName = "tls-prod.pem"
        else:
            targetName = "seerep"
            certName = "tls.pem"

        # server with certs
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        with open(os.path.join(__location__, "../../../certs/" + certName), "rb") as f:
            root_cert = f.read()
        server = targetName + ".robot.10.249.3.13.nip.io:31723"
        creds = grpc.ssl_channel_credentials(root_cert)
        channel = grpc.secure_channel(server, creds)

    elif target == "local":
        # server without certs
        server = "localhost:9090"
        channel = grpc.insecure_channel(server, options=options)
    else:
        channel = grpc.insecure_channel(target, options=options)

    return channel


# based on https://github.com/jpvanhal/inflection
def to_snake_case(string: str):
    """
    Tries to convert a common string to snake case.

    """
    string = re.sub(r"((?<=[a-z0-9])[A-Z]|(?!^)(?<!_)[A-Z](?=[a-z]))", r"_\1", string)
    string = string.replace("-", "_")
    return string.lower()


def dict_snake_case_keys(d: Dict[str, Any]) -> Any:
    """
    Recursively converts all keys of a dict to snake case.

    This recurses into lists and dicts, but not into other data structures.
    Only keys of type `str` will be converted.
    This implementation deepcopies the input parameter.

    Args:
        d: dictionary for conversion
    Returns:
        the resulting dictionary
    """
    obj = deepcopy(d)
    if isinstance(obj, list):
        for idx, elem in enumerate(obj):
            obj[idx] = dict_snake_case_keys(elem)
    if not isinstance(obj, dict):
        return obj
    for k, v in d.items():
        obj.pop(k)
        v = dict_snake_case_keys(v)
        if isinstance(k, str):
            obj[to_snake_case(k)] = v
        else:
            obj[k] = v

    return obj
