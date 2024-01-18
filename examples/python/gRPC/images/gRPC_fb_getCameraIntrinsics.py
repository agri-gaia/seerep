#!/usr/bin/env python3

from typing import Optional

import flatbuffers
from grpc import Channel
from seerep.fb import CameraIntrinsics
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import createCameraIntrinsicsQuery, getProject


def get_camintrins(
    ciuuid: str = "fa2f27e3-7484-48b0-9f21-ec362075baca",
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> Optional[CameraIntrinsics.CameraIntrinsics]:

    builder = flatbuffers.Builder(1000)

    # 1. Get all projects from the server when no target specified
    if target_proj_uuid is None:
        # this finishes the builder and returns the decoded uuid
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")
        if target_proj_uuid is None:
            print("valid project doesn't exist! Please execute gRPC_fb_addCameraIntrinsics.py beforehand.")
            return None
    camintrins_query = createCameraIntrinsicsQuery(builder, ciuuid, target_proj_uuid)
    builder.Finish(camintrins_query)

    buf = builder.Output()

    stub = ci_service.CameraIntrinsicsServiceStub(grpc_channel)

    ret = stub.GetCameraIntrinsics(bytes(buf))

    retrieved_ci = CameraIntrinsics.CameraIntrinsics.GetRootAs(ret)

    return retrieved_ci


if __name__ == "__main__":
    camintrins = get_camintrins()
    # printing the uuid of the retrieved camera intrinsics to verify the result of the query
    print(
        f"the camera instrinsics with uuid {camintrins.Header().UuidMsgs().decode('utf-8')} was retrieved from the project with the uuid {camintrins.Header().UuidProject().decode('utf-8')}"
    )
    # print the distortion of the retrieved camera intrinsics
    print(f"camera instrinsics distortion array: {camintrins.DistortionAsNumpy()}")
