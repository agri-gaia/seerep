from typing import Dict, List, Tuple

import fb_to_dict
from gRPC.images import gRPC_pb_sendLabeledImageGrid as send_grid
from gRPC.point import gRPC_fb_queryPoints as query_points
from gRPC.point import gRPC_fb_sendPointsBasedOnImageInstances as send_points
from seerep.fb import PointStamped


def test_sendAndQueryPoints(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup

    # send images to the project
    send_grid.send_labeled_image_grid(proj_uuid, grpc_channel)

    # send points to the project
    sent_points_dict: Dict = send_points.send_points(proj_uuid, grpc_channel)

    sent_point_cnt = 0
    # count sent points
    for val in sent_points_dict.values():
        sent_point_cnt += len(val)

    # extract the sent points to a list
    sent_plist: List[PointStamped.PointStamped, Tuple[float, float, float]] = sorted(
        [
            (label_p[1], label_p[2])
            for label_p in val
            for val in sent_points_dict.values()
        ],
        key=lambda p: p[0].Header().UuidMsgs().decode(),
    )

    # query points from the project
    queried_points: List[PointStamped.PointStamped] = sorted(
        query_points.get_points(proj_uuid, grpc_channel),
        key=lambda p: p.Header().UuidMsgs().decode(),
    )

    assert len(sent_plist) == len(queried_points)

    # print(f"Sent coords: {sent_plist[0][1][1]}")

    print(fb_to_dict.fb_obj_to_dict(queried_points[0].Header()))
    print(fb_to_dict.fb_obj_to_dict(queried_points[0].Point()))
    for idx in range(queried_points[0].LabelsGeneralLength()):
        print(fb_to_dict.fb_obj_to_dict(queried_points[0].LabelsGeneral(idx)))

    print(queried_points[0].Attribute(0).ValueType())
    print(sent_plist[0][0].Attribute(0).Value().String(0))

    # check if the queried points are the same as the sent points
    # for idx in range(len(queried_points)):
    #     assert fb_to_dict.fb_obj_to_dict(sent_plist[idx]) == fb_to_dict.fb_obj_to_dict(queried_points[idx])
