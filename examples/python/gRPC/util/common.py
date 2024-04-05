import os
import re
from typing import Dict

import grpc


def get_gRPC_channel(target="local"):
    # set the max message size to 1GB (half the size of the server)
    options = [
        ("grpc.max_send_message_length", 1 * 1024 * 1024 * 1024),
        ("grpc.max_receive_message_length", 1 * 1024 * 1024 * 1024),
    ]

    if target in ("prod", "dev"):
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
# function to pass to boltons.iterutils.remap
def remap_to_snake_case(p, k, v):
    if isinstance(k, str):
        k = re.sub(r"((?<=[a-z0-9])[A-Z]|(?!^)(?<!_)[A-Z](?=[a-z]))", r"_\1", k)
        k = k.replace("-", "_")
        return k.lower(), v
    return k, v


# function to pass to boltons.iterutils.remap
def remap_keys(p, k, v, mapping: Dict[str, str] = {}):
    k = mapping.get(k, k)
    if k in mapping:
        print(k)
    return k, v


# function to pass to boltons.iterutils.remap
def trunc_floats(p, k, v, ignore_after: int = None):
    """
    This Function can be provided to boltons.iterutils.remap
    and truncates `ignore_after` decimal places after the specified position behind the dot.

    Args:
        p, k, v: path, key, value for boltons.iterutils.remap
        ignore_after: specifies at what position after the dot to strip the decimals,
                      None means don't truncate the values

    Returns: (k, v) as specified by boltons.iterutils.remap, where v is modified if v is of type float
    """
    if ignore_after is not None and isinstance(v, float):
        # use string formatting to chop the float
        s = f"{{:.{ignore_after}f}}".format(v)
        v = float(s)
    return k, v
