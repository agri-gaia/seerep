# NOTE: This file is referenced in the following mkdocs files:
#   python-helpers.md
# If any line changes on this file occur, those files may have to be updated as
# well
import json
import os
import subprocess as sp
import tempfile
from enum import Enum
from pathlib import Path
from typing import Dict, Final

ROS_SCHEMA_PKG: Final[str] = "seerep_msgs"
ROS_SCHEMA_SUBDIR: Final[str] = "fbs"


class SchemaFileNames(Enum):
    ATTRIBUTES_STAMPED: Final[str] = "attributes_stamped.fbs"
    BOUNDINGBOX: Final[str] = "boundingbox.fbs"
    CAMERA_INTRINSICS: Final[str] = "camera_intrinsics.fbs"
    CAMERA_INTRINSICS_QUERY: Final[str] = "camera_intrinsics_query.fbs"
    DATASET_UUID_LABEL: Final[str] = "dataset_uuid_label.fbs"
    DATATYPE: Final[str] = "datatype.fbs"
    EMPTY: Final[str] = "empty.fbs"
    FRAME_QUERY: Final[str] = "frame_query.fbs"
    GEODETICCOORDINATES: Final[str] = "geodeticCoordinates.fbs"
    HEADER: Final[str] = "header.fbs"
    IMAGE: Final[str] = "image.fbs"
    LABEL: Final[str] = "label.fbs"
    LABEL_CATEGORY: Final[str] = "label_category.fbs"
    LABELS: Final[str] = "labels.fbs"
    POINT2D: Final[str] = "point2d.fbs"
    POINT_CLOUD_2: Final[str] = "point_cloud_2.fbs"
    POINT: Final[str] = "point.fbs"
    POINT_FIELD: Final[str] = "point_field.fbs"
    POINT_STAMPED: Final[str] = "point_stamped.fbs"
    POLYGON2D: Final[str] = "polygon2d.fbs"
    PROJECTCREATION: Final[str] = "projectCreation.fbs"
    PROJECT_INFO: Final[str] = "project_info.fbs"
    PROJECT_INFOS: Final[str] = "project_infos.fbs"
    QUATERNION: Final[str] = "quaternion.fbs"
    QUERY: Final[str] = "query.fbs"
    QUERY_INSTANCE: Final[str] = "query_instance.fbs"
    REGION_OF_INTEREST: Final[str] = "region_of_interest.fbs"
    SERVER_RESPONSE: Final[str] = "server_response.fbs"
    SPARQL_QUERY: Final[str] = "sparql_query.fbs"
    STRING_VECTOR: Final[str] = "string_vector.fbs"
    TIME_INTERVAL: Final[str] = "time_interval.fbs"
    TIMESTAMP: Final[str] = "timestamp.fbs"
    TRANSFORM: Final[str] = "transform.fbs"
    TRANSFORM_STAMPED: Final[str] = "transform_stamped.fbs"
    TRANSFORM_STAMPED_QUERY: Final[str] = "transform_stamped_query.fbs"
    UNION_MAP_ENTRY: Final[str] = "union_map_entry.fbs"
    UUID_DATATYPE_PAIR: Final[str] = "uuid_datatype_pair.fbs"
    UUID_DATATYPE_WITH_CATEGORY: Final[str] = "uuid_datatype_with_category.fbs"
    UUIDS_PER_PROJECT: Final[str] = "uuids_per_project.fbs"
    UUIDS_PROJECT_PAIR: Final[str] = "uuids_project_pair.fbs"
    VECTOR3: Final[str] = "vector3.fbs"


def catkin_find_schema_dir(ros_pkg_name: str, sub_dir: str) -> Path:
    """
    Tries to find the schema directory on the system using `catkin locate`.

    Args:
        ros_pkg_name: The name of the ros package containing the relevant
        schema files
        sub_dir: The name of the subdir of the package dir containing the
        relevant schema files

    Returns:
        The schema directory on the system if found.

    Raises:
        FileNotFoundError: If the path on the system is not present.
        ChildProcessError: If `catkin locate` returns something on stderr
        or failed otherwise
    """

    catkin_proc = sp.Popen(
        ["catkin", "locate", ros_pkg_name],
        encoding="utf-8",
        stdout=sp.PIPE,
        stderr=sp.PIPE,
    )
    catkin_out, catkin_err = catkin_proc.communicate()

    if catkin_proc.returncode != 0 or catkin_err != "":
        raise ChildProcessError(f"Catkin call errored! {catkin_err}")

    # splitlines needs to be used otherwise catkin_out contains \n at the end
    fbs_path: Path = Path(catkin_out.splitlines()[0]) / sub_dir

    if not fbs_path.is_dir():
        raise FileNotFoundError(
            f"The path for seerep flatbuffers schema files is not present! "
            f"Should be {fbs_path}"
        )

    return fbs_path


def fb_flatc_dict(fb_obj: bytearray, schema_file_name: SchemaFileNames) -> Dict:
    """
    Converts a binary flatbuffers object to a python dictionary using it's IDL
    file.

    This function should only be used for debugging or testing purposes, as it
    alleviates the advantage of flatbuffers lessening the amount of copied data.

    This implementation uses temporary files in /tmp for conversion.

    Args:
        fb_obj: The bytearray object as returned by builder.Output().
        schema_file_name: The to `fb_obj` corresponding datatype in the
        `SchemaFileNames` format

    Returns:
        A python dictionary containing the objects attribute information.

    Raises:
        FileNotFoundError: If the schema file on the system couldn't be found.
        ChildProcessError: If something went wrong using the flatc subcommand.
    """
    schema_path = (
        catkin_find_schema_dir(ROS_SCHEMA_PKG, ROS_SCHEMA_SUBDIR)
        / schema_file_name.value
    )

    if not schema_path.is_file():
        raise FileNotFoundError(
            f"The schema file at {schema_path} does not exist!"
        )

    try:
        with tempfile.NamedTemporaryFile(delete=False) as f:
            f.write(fb_obj)
            flatc_proc = sp.Popen(
                [
                    "flatc",
                    "--json",
                    "--raw-binary",
                    "--strict-json",
                    schema_path,
                    "--",
                    f.name,
                ],
                cwd="/tmp",
            )
            tmp_file = Path(f.name)
            tmp_json = Path(f.name + ".json")

        sp_ret_code = flatc_proc.wait()
        os.sync()

        if sp_ret_code != 0 or not tmp_json.is_file():
            raise ChildProcessError(
                f"A problem occured during flatbuffers object to json "
                f"conversion using flatc! "
                f"The file {tmp_json} was not properly created. Has the fbs "
                f"schema file the `root_type` directive set?"
            )

        with open(tmp_json, "r") as tmp_f:
            json_dict = json.loads(tmp_f.read())
    finally:
        tmp_file.unlink(missing_ok=True)
        tmp_json.unlink(missing_ok=True)

    return json_dict
