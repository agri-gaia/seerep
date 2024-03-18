import json
import os
import subprocess as sp
import tempfile as tf
from pathlib import Path
from typing import Dict, Final

SCHEMA_FOLDER: Final[str] = "seerep_msgs/fbs/"


def fb_flatc_dict(fb_obj: bytearray, schema_file_name: str) -> Dict:
    """
    Converts a binary flatbuffers object to a python dictionary using it's IDL file.
    This implementation uses temporary files in /tmp for conversion.

    Args:
        fb_obj: The bytearray object as returned by builder.Output().
        schema_file_name: The filename of the fb schema file.

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
