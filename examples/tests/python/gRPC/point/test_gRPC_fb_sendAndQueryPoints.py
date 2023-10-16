from typing import Dict, List, Tuple, Type

import fb_to_dict
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

    # check if the queried points are the same as the sent points
    for idx in range(len(queried_points)):
        assert fb_to_dict.fb_obj_to_dict(
            sent_plist[idx], union_type_mapping=union_type_mapping
        ) == fb_to_dict.fb_obj_to_dict(
            queried_points[idx], union_type_mapping=union_type_mapping
        )
