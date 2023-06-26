#!/usr/bin/env python3

import uuid

import flatbuffers
from seerep.fb import CameraIntrinsics
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createCameraIntrinsicsQuery,
    createHeader,
    createRegionOfInterest,
    createTimeStamp,
    getProject,
)


def get_ciuuid():
    builder = flatbuffers.Builder(1000)
    # Default server is localhost !
    channel = get_gRPC_channel()

    # 1. Get all projects from the server
    projectuuid = getProject(builder, channel, "testproject")

    ciuuid = str(uuid.uuid4())
    print("Camera Intrinsics will be saved against the uuid: ", ciuuid)

    # 2. Check if the defined project exist; if not exit
    if not projectuuid:
        print("project doesn't exist!")
        exit()

    # 3. Get gRPC service object
    stub = ci_service.CameraIntrinsicsServiceStub(channel)

    # Create all necessary objects for the query
    ts = createTimeStamp(builder, 4, 3)
    header = createHeader(builder, ts, "map", projectuuid, ciuuid)
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

    stub.TransferCameraIntrinsics(bytes(buf))

    # Fetch the saved CI
    builder = flatbuffers.Builder(1000)

    ci_query = createCameraIntrinsicsQuery(builder, ciuuid, projectuuid)

    builder.Finish(ci_query)
    buf = builder.Output()

    ret = stub.GetCameraIntrinsics(bytes(buf))

    retrieved_ci = CameraIntrinsics.CameraIntrinsics.GetRootAs(ret)

    # printing the uuid of the retrieved camera intrinsics
    print(retrieved_ci.Header().UuidMsgs().decode("utf-8"))
    return ciuuid
