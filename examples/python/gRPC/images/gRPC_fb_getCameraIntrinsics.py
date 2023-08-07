#!/usr/bin/env python3

import flatbuffers
from seerep.fb import CameraIntrinsics
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import createCameraIntrinsicsQuery


def get_camintrins(target_proj_uuid, ciuuid, grpc_channel=get_gRPC_channel()):
    builder = flatbuffers.Builder(1000)
    camintrins_query = createCameraIntrinsicsQuery(builder, ciuuid, target_proj_uuid)
    builder.Finish(camintrins_query)

    buf = builder.Output()

    stub = ci_service.CameraIntrinsicsServiceStub(grpc_channel)

    ret = stub.GetCameraIntrinsics(bytes(buf))

    retrieved_ci = CameraIntrinsics.CameraIntrinsics.GetRootAs(ret)

    # printing the uuid of the retrieved camera intrinsics
    print(retrieved_ci.Header().UuidMsgs().decode("utf-8"))

    return retrieved_ci
