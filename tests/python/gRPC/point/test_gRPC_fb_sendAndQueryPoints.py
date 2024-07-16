# test file for
#   gRPC_fb_queryPoints.py
#   gRPC_fb_sendPointsBasedOnImageInstances.py
# requires:
#   gRPC_pb_sendLabeledImageGrid.py
from typing import List

from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid
from gRPC.point import gRPC_fb_queryPoints as query_points
from gRPC.point import gRPC_fb_sendPointsBasedOnImageInstances as send_points
from seerep.fb import (
    PointStamped,
)
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


# todo: test query dependant on tf
def test_sendAndQueryPoints(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    # send images to the project
    send_grid.send_labeled_image_grid(proj_uuid, grpc_channel)

    # extract the sent points to a list
    sent_points: List[PointStamped.PointStamped] = sorted(
        [
            fb_flatc_dict(p, SchemaFileNames.POINT_STAMPED)
            for ls in send_points.send_points_raw(
                proj_uuid, grpc_channel
            ).values()
            for p in ls
        ],
        key=lambda p: p["header"]["uuid_msgs"],
    )

    # query points from the project
    queried_points: List[PointStamped.PointStamped] = sorted(
        [
            fb_flatc_dict(point, SchemaFileNames.POINT_STAMPED)
            for point in query_points.get_points_raw(proj_uuid, grpc_channel)
        ],
        key=lambda p: p["header"]["uuid_msgs"],
    )

    assert len(sent_points) == len(queried_points)

    # Fix current discrepancy between the sent and queried points in the
    # attributes, because the queried points contain the header additionally

    # remove the header entries of the queried points from the attributes
    for p in queried_points:
        # sort attribute lists
        p["attribute"] = sorted(p["attribute"], key=lambda a: a["key"])

        p_l = p["attribute"]
        idx = 0
        while idx < len(p_l):
            if p_l[idx]["key"] in [
                "frame_id",
                "header_seq",
                "stamp_nanos",
                "stamp_seconds",
            ]:
                p_l.pop(idx)
            else:
                idx += 1

    # sort attribute lists
    for p in sent_points:
        p["attribute"] = sorted(p["attribute"], key=lambda a: a["key"])

    assert sent_points == queried_points
