import json
import os
import subprocess as sp
import tempfile as tf
from pathlib import Path
from typing import Dict, Final

SCHEMA_FOLDER: Final[str] = "/seerep/src/seerep_msgs/fbs/"


class SchemaFileNames:
    ATTRIBUTES_STAMPED: Final[str] = "attributes_stamped.fbs"
    BOUNDINGBOX2D: Final[str] = "boundingbox2d.fbs"
    BOUNDINGBOX2D_LABELED: Final[str] = "boundingbox2d_labeled.fbs"
    BOUNDINGBOX2D_LABELED_WITH_CATEGORY: Final[str] = "boundingbox2d_labeled_with_category.fbs"
    BOUNDINGBOX2D_STAMPED: Final[str] = "boundingbox2d_stamped.fbs"
    BOUNDINGBOXES2D_LABELED_STAMPED: Final[str] = "boundingboxes2d_labeled_stamped.fbs"
    BOUNDINGBOXES_LABELED_STAMPED: Final[str] = "boundingboxes_labeled_stamped.fbs"
    BOUNDINGBOX: Final[str] = "boundingbox.fbs"
    BOUNDINGBOX_LABELED: Final[str] = "boundingbox_labeled.fbs"
    BOUNDINGBOX_LABELED_WITH_CATEGORY: Final[str] = "boundingbox_labeled_with_category.fbs"
    BOUNDINGBOX_STAMPED: Final[str] = "boundingbox_stamped.fbs"
    CAMERA_INTRINSICS: Final[str] = "camera_intrinsics.fbs"
    CAMERA_INTRINSICS_QUERY: Final[str] = "camera_intrinsics_query.fbs"
    CATEGORIES: Final[str] = "categories.fbs"
    DATATYPE: Final[str] = "datatype.fbs"
    EMPTY: Final[str] = "empty.fbs"
    FRAME_INFOS: Final[str] = "frame_infos.fbs"
    FRAME_QUERY: Final[str] = "frame_query.fbs"
    GEODETICCOORDINATES: Final[str] = "geodeticCoordinates.fbs"
    HEADER: Final[str] = "header.fbs"
    IMAGE: Final[str] = "image.fbs"
    LABEL: Final[str] = "label.fbs"
    LABELS: Final[str] = "labels.fbs"
    LABELS_WITH_CATEGORY: Final[str] = "labels_with_category.fbs"
    LABELS_WITH_INSTANCE_WITH_CATEGORY: Final[str] = "labels_with_instance_with_category.fbs"
    LABEL_WITH_INSTANCE: Final[str] = "label_with_instance.fbs"
    POINT2D: Final[str] = "point2d.fbs"
    POINT_CLOUD_2: Final[str] = "point_cloud_2.fbs"
    POINT: Final[str] = "point.fbs"
    POINT_FIELD: Final[str] = "point_field.fbs"
    POINT_STAMPED: Final[str] = "point_stamped.fbs"
    POLYGON2D: Final[str] = "polygon2d.fbs"
    POLYGON: Final[str] = "polygon.fbs"
    POLYGON_STAMPED: Final[str] = "polygon_stamped.fbs"
    POSE: Final[str] = "pose.fbs"
    POSE_STAMPED: Final[str] = "pose_stamped.fbs"
    PROJECTCREATION: Final[str] = "projectCreation.fbs"
    PROJECT_INFO: Final[str] = "project_info.fbs"
    PROJECT_INFOS: Final[str] = "project_infos.fbs"
    QUATERNION: Final[str] = "quaternion.fbs"
    QUATERNION_STAMPED: Final[str] = "quaternion_stamped.fbs"
    QUERY: Final[str] = "query.fbs"
    QUERY_INSTANCE: Final[str] = "query_instance.fbs"
    REGION_OF_INTEREST: Final[str] = "region_of_interest.fbs"
    SERVER_RESPONSE: Final[str] = "server_response.fbs"
    SPARQL_QUERY: Final[str] = "sparql_query.fbs"
    TIME_INTERVAL: Final[str] = "time_interval.fbs"
    TIMESTAMP: Final[str] = "timestamp.fbs"
    TRANSFORM: Final[str] = "transform.fbs"
    TRANSFORM_STAMPED: Final[str] = "transform_stamped.fbs"
    TRANSFORM_STAMPED_QUERY: Final[str] = "transform_stamped_query.fbs"
    TWIST: Final[str] = "twist.fbs"
    TWIST_STAMPED: Final[str] = "twist_stamped.fbs"
    TWIST_WITH_COVARIANCE: Final[str] = "twist_with_covariance.fbs"
    TWIST_WITH_COVARIANCE_STAMPED: Final[str] = "twist_with_covariance_stamped.fbs"
    UNION_MAP_ENTRY: Final[str] = "union_map_entry.fbs"
    UUID_DATATYPE_PAIR: Final[str] = "uuid_datatype_pair.fbs"
    UUID_DATATYPE_WITH_CATEGORY: Final[str] = "uuid_datatype_with_category.fbs"
    UUIDS_PER_PROJECT: Final[str] = "uuids_per_project.fbs"
    UUIDS_PROJECT_PAIR: Final[str] = "uuids_project_pair.fbs"
    VECTOR3: Final[str] = "vector3.fbs"
    VECTOR3_STAMPED: Final[str] = "vector3_stamped.fbs"


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

    if not schema_path.is_file():
        raise FileNotFoundError(f"The schema file at {schema_path} does not exist!")

    try:
        with tf.NamedTemporaryFile(delete=False) as tmp_f:
            tmp_f.write(fb_obj)
            temp_fname = tmp_f.name
            flatc_proc = sp.Popen(
                ["flatc", "--json", "--raw-binary", "--strict-json", schema_path, "--", temp_fname], cwd="/tmp"
            )

        sp_ret_code = flatc_proc.wait()

        temp_json = temp_fname + ".json"

        if sp_ret_code != 0 or not Path(temp_json).is_file():
            raise ChildProcessError(
                f"A problem occured during flatbuffers object to json conversion using flatc!\
                The file {temp_json} was not properly created. Has the fbs schema file the `root_type` directive set?"
            )

        with open(temp_json, "r") as tmp_f:
            json_dict = json.loads(tmp_f.read())
    finally:
        try:
            os.remove(temp_fname)
            os.remove(temp_json)
        except (FileNotFoundError, UnboundLocalError):
            pass

    return json_dict
