# test file for
#  gRPC_pb_addCameraIntrinsics.py
#  gRPC_pb_getCameraIntrinsics.py
import logging

from gRPC.images import grpc_pb_addCameraIntrinsics as add_intrinsics
from gRPC.images import grpc_pb_getCameraIntrinsics as get_intrinsics


def test_gRPC_pb_addAndGetCamins(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup
    logging.info(f"Testing project: {proj_name}; {proj_uuid}")

    # test for image uuid, general label and the general label confidence of corresponding entry in 2d array
    sent_intrinsics = add_intrinsics.add_camintrins(grpc_channel, proj_uuid)
    queried_intrinsics = get_intrinsics.query_camintrins(
        proj_uuid, sent_intrinsics.header.uuid_msgs, grpc_channel
    )

    assert len(sent_intrinsics) == len(queried_intrinsics)
    sent_intrinsics = sorted(sent_intrinsics, key=lambda i: i.header.uuid_msgs)
    queried_intrinsics = sorted(queried_intrinsics, key=lambda i: i.header.uuid_msgs)
    for idx, intrinsics in enumerate(queried_intrinsics):
        assert sent_intrinsics[idx].header.uuid_msgs == intrinsics.header.uuid_msgs
        assert sent_intrinsics[idx].cameraIntrinsics == intrinsics.cameraIntrinsics
