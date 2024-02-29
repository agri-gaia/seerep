# test file for
#   gRPC_fb_queryPoints.py
#   gRPC_fb_sendPointsBasedOnImageInstances.py
# requires:
#   gRPC_pb_sendLabeledImageGrid.py
from typing import Dict, List, Tuple, Type

from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid
from gRPC.point import gRPC_fb_queryPoints as query_points
from gRPC.point import gRPC_fb_sendPointsBasedOnImageInstances as send_points
from seerep.fb import (
    Boolean,
    Datatypes,
    Double,
    Integer,
    PointStamped,
    String,
    UnionMapEntry,
)
from seerep.util import fb_to_dict


# todo: test query dependant on tf
def test_sendAndQueryPoints(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    # send images to the project
    send_grid.send_labeled_image_grid(proj_uuid, grpc_channel)

    # send points to the project
    sent_points_dict: Dict = send_points.send_points(proj_uuid, grpc_channel)

    # extract the sent points to a list
    sent_plist: List[PointStamped.PointStamped] = sorted(
        [p for ls in sent_points_dict.values() for p in ls],
        key=lambda p: p.Header().UuidMsgs().decode(),
    )

    # query points from the project
    queried_points: List[PointStamped.PointStamped] = sorted(
        query_points.get_points(proj_uuid, grpc_channel),
        key=lambda p: p.Header().UuidMsgs().decode(),
    )

    assert len(sent_plist) == len(queried_points)

    # build union type mapping
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

    # Fix current discrepancy between the sent and queried points in the attributes,
    # because the queried points contain the header additionally
    # convert fb objects to dicts
    sent_pdicts_list = [fb_to_dict.fb_obj_to_dict(p, union_type_mapping=union_type_mapping) for p in sent_plist]
    queried_pdicts_list = [fb_to_dict.fb_obj_to_dict(p, union_type_mapping=union_type_mapping) for p in queried_points]

    # remove the header entries of the queried points from the attributes
    for p in queried_pdicts_list:
        # sort attribute lists
        p["Attribute"] = sorted(p["Attribute"], key=lambda a: a["Key"])

        p_l = p["Attribute"]
        idx = 0
        while idx < len(p_l):
            if p_l[idx]["Key"] in [
                "frame_id",
                "header_seq",
                "stamp_nanos",
                "stamp_seconds",
            ]:
                p_l.pop(idx)
            else:
                idx += 1

    # sort attribute lists
    for p in sent_pdicts_list:
        p["Attribute"] = sorted(p["Attribute"], key=lambda a: a["Key"])

    # check if the queried points are the same as the sent points
    assert len(queried_pdicts_list) == len(sent_pdicts_list)
    assert queried_pdicts_list == sent_pdicts_list
