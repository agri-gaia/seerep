# test file for
#   gRPC_pb_addCameraIntrinsics.py
#   gRPC_pb_getCameraIntrinsics.py
import logging

from gRPC.images import gRPC_pb_addCameraIntrinsics as add_intrinsics
from gRPC.images import gRPC_pb_getCameraIntrinsics as get_intrinsics


def protobuf_obj_to_dict(obj, d=None):
    if d is None:
        d = {}
    for descriptor in obj.DESCRIPTOR.fields:
        val = getattr(obj, descriptor.name)
        if descriptor.type == descriptor.TYPE_MESSAGE:
            inner_dict = {}
            d[descriptor.name] = inner_dict
            protobuf_obj_to_dict(val, inner_dict)
        elif descriptor.type == descriptor.TYPE_ENUM:
            d[descriptor.name] = descriptor.enum_type.values_by_number[val].name
        else:
            d[descriptor.name] = getattr(obj, descriptor.name)
    return d


def test_gRPC_pb_addAndGetCamins(grpc_channel, project_setup):
    proj_name, proj_uuid = project_setup
    logging.info(f"Testing project: {proj_name}; {proj_uuid}")

    # test for image uuid, general label and the general label confidence of
    # corresponding entry in 2d array
    sent_intrinsics = add_intrinsics.add_camintrins(
        target_proj_uuid=proj_uuid, grpc_channel=grpc_channel
    )
    queried_intrinsics = get_intrinsics.query_camintrins(
        sent_intrinsics.header.uuid_msgs, proj_uuid, grpc_channel
    )

    print(sent_intrinsics.intrinsic_matrix)

    assert protobuf_obj_to_dict(sent_intrinsics) == protobuf_obj_to_dict(
        queried_intrinsics
    )
