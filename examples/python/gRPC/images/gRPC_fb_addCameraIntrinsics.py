#!/usr/bin/env python3
import uuid

import flatbuffers
from seerep.fb import CameraIntrinsics
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createHeader,
    createRegionOfInterest,
    createTimeStamp,
    getProject,
)


# Default server is localhost !
def add_camintrins(grpc_channel=get_gRPC_channel(), target_proj_uuid=None):

    builder = flatbuffers.Builder(1000)

    # 1. Get all projects from the server when no target specified
    if not target_proj_uuid:
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")

    ciuuid = str(uuid.uuid4())
    print("Camera Intrinsics will be saved against the uuid:", ciuuid)

    # 2. Check if the defined project exist; if not exit
    if not target_proj_uuid:
        print("project doesn't exist!")
        exit()

    # 3. Get gRPC service object
    stub = ci_service.CameraIntrinsicsServiceStub(grpc_channel)

    # Create all necessary objects for the query
    ts = createTimeStamp(builder, 4, 3)
    header = createHeader(builder, ts, "map", target_proj_uuid, ciuuid)
    roi = createRegionOfInterest(builder, 3, 5, 6, 7, True)


    distortion_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    rect_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    intrins_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    proj_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    ci = createCameraIntrinsics(
        builder, header, 3, 4, "plumb_bob", distortion_matrix, intrins_matrix, rect_matrix, proj_matrix, 4, 5, roi, 5
    )
    builder.Finish(ci)

    buf = builder.Output()

    ret = stub.TransferCameraIntrinsics(bytes(buf))
    return CameraIntrinsics.CameraIntrinsics.GetRootAs(buf)


if __name__ == "__main__":
    print(dir(add_camintrins()))
