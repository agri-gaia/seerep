import os
import re
from typing import Dict, Hashable

import grpc


def get_gRPC_channel(target: str = "local") -> grpc.Channel:
    """
    Returns a gRPC channel to the specified target.

    Args:
        target: The target to connect to, can be "local", "prod", "dev" or an \
            IP address with a port\
            in the format "IP:PORT"

    Returns:
        A gRPC channel to the specified target
    """
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
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__))
        )
        with open(
            os.path.join(__location__, "../../../certs/" + certName), "rb"
        ) as f:
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


def remap_to_snake_case(p: tuple, k: Hashable, v):
    """
    Remaps all string type keys of a dictionary to snake_case.
    This is a function to pass to boltons.iterutils.remap.
    Based on https://github.com/jpvanhal/inflection.

    Args:
        p, k, v: path, key, value for boltons.iterutils.remap

    Returns:
        (k, v) as specified by boltons.iterutils.remap, where k is modified
        if k is of type string
    """
    if isinstance(k, str):
        k = re.sub(r"((?<=[a-z0-9])[A-Z]|(?!^)(?<!_)[A-Z](?=[a-z]))", r"_\1", k)
        k = k.replace("-", "_")
        return k.lower(), v
    return k, v


def remap_keys(p: tuple, k: Hashable, v, mapping: Dict[str, str] = {}):
    """
    Remaps all keys of a dictionary according to the mapping provided.
    This is a function to pass to boltons.iterutils.remap.

    Args:
        p, k, v: path, key, value for boltons.iterutils.remap
        mapping: a dictionary that maps the old key to the new key

    Returns:
        (k, v) as specified by boltons.iterutils.remap, where k is modified
        if k is in the mapping
    """
    k = mapping.get(k, k)
    return k, v


def trunc_floats(p: tuple, k: Hashable, v, ignore_after: int = None):
    """
    This Function can be provided to boltons.iterutils.remap
    and truncates `ignore_after` decimal places after the specified position
    behind the dot.

    Args:
        p, k, v: path, key, value for boltons.iterutils.remap
        ignore_after: specifies at what position after the dot to strip the
        decimals, None means don't truncate the values

    Returns:
        (k, v) as specified by boltons.iterutils.remap, where v is modified
        if v is of type float
    """
    if ignore_after is not None and isinstance(v, float):
        # use string formatting to chop the float
        s = f"{{:.{ignore_after}f}}".format(v)
        v = float(s)
    return k, v
