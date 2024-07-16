#!/usr/bin/env python3
import uuid
from typing import Optional

import flatbuffers
from google.protobuf import empty_pb2
from grpc import Channel
from seerep.fb import CameraIntrinsics
from seerep.fb import camera_intrinsics_service_grpc_fb as ci_service
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createHeader,
    createRegionOfInterest,
    createTimeStamp,
)


# Default server is localhost !
def add_camintrins_raw(
    ciuuid: Optional[str] = "fa2f27e3-7484-48b0-9f21-ec362075baca",
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> Optional[bytearray]:
    """
    Creates a example cameraintrinsics object and sends it to a SEEREP server
    instance.

    Args:
        ciuuid: the uuid of the CameraIntrinsics object to be send
        target_proj_uuid: the project uuid to which project to send the
        CameraIntrinsics object to grpc_channel: the grpc channel to a SEEREP
        server

    Returns: bytearray which contains the serialized type
    seerep.fb.CameraInstrinsics.CameraIntrinsics
    """
    if ciuuid is None:
        ciuuid = str(uuid.uuid4())

    builder = flatbuffers.Builder(1000)

    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    # 3. Check if we have an existing test project, if not, one is created.
    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "testproject":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            response = stubMeta.CreateProject(
                projectCreation_pb2.ProjectCreation(
                    name="testproject", mapFrameId="map"
                )
            )
            target_proj_uuid = response.uuid

    # 2. Check if the defined project exist; if not return None
    if target_proj_uuid is None:
        print("could not create project and add camera intrinsics to it!")
        return None

    # 3. Get gRPC service object
    stub = ci_service.CameraIntrinsicsServiceStub(grpc_channel)

    # Create all necessary objects for the query
    ts = createTimeStamp(builder, 1, 2)
    header = createHeader(builder, ts, "map", target_proj_uuid, ciuuid)
    roi = createRegionOfInterest(builder, 3, 5, 6, 7, True)

    distortion_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    rect_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    intrins_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    proj_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    ci = createCameraIntrinsics(
        builder,
        header,
        3,
        4,
        "plumb_bob",
        distortion_matrix,
        intrins_matrix,
        rect_matrix,
        proj_matrix,
        4,
        5,
        roi,
        5,
    )
    builder.Finish(ci)

    buf = builder.Output()

    stub.TransferCameraIntrinsics(bytes(buf))
    return buf


def add_camintrins(
    ciuuid: Optional[str] = "fa2f27e3-7484-48b0-9f21-ec362075baca",
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
) -> Optional[CameraIntrinsics.CameraIntrinsics]:
    return CameraIntrinsics.CameraIntrinsics.GetRootAs(
        add_camintrins_raw(ciuuid, target_proj_uuid, grpc_channel)
    )


if __name__ == "__main__":
    caminstrins_obj = add_camintrins()
    # for verification print the uuid of the added camera intrinsics
    print(
        f"camera instrinsics were saved with the uuid \
        {caminstrins_obj.Header().UuidMsgs().decode('utf-8')} on the project \
        with the uuid \
        {caminstrins_obj.Header().UuidProject().decode('utf-8')}"
    )
    # and the contents of the distortion array
    print(
        f"camera instrinsics distortion matrix: "
        f"{caminstrins_obj.DistortionAsNumpy()}"
    )
