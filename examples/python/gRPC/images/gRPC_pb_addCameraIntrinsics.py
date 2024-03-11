#!/usr/bin/env python3
import uuid
from typing import Optional

from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import camera_intrinsics_pb2 as cameraintrinsics
from seerep.pb import camera_intrinsics_service_pb2_grpc as camintrinsics_service
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2
from seerep.util.common import get_gRPC_channel


# default grpc_channel is localhost:9090
def add_camintrins(
    ciuuid: Optional[str] = "fa2f27e3-7484-48b0-9f21-ec362075baca",
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
):
    # 1. Get gRPC service objects
    stub = camintrinsics_service.CameraIntrinsicsServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    # 2. Get all projects from the server
    response = stubMeta.GetProjects(empty_pb2.Empty())

    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)

    if target_proj_uuid is None:
        # 2. Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "testproject":
                target_proj_uuid = project.uuid

# 3. Check if we have an existing test project, if not, we stop here
projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid + "\n")
    if project.name == "testproject":
        projectuuid = "f2e3c8d9-e9f0-4687-858c-8899a07038f4"

    if ciuuid is None:
        ciuuid = str(uuid.uuid4())

    camin = cameraintrinsics.CameraIntrinsics()

    camin.header.stamp.seconds = 4
    camin.header.stamp.nanos = 3

    camin.header.frame_id = "camintrinsics"

    camin.header.uuid_project = target_proj_uuid
    camin.header.uuid_msgs = ciuuid

    camin.region_of_interest.x_offset = 2
    camin.region_of_interest.y_offset = 1
    camin.region_of_interest.height = 5
    camin.region_of_interest.width = 4
    camin.region_of_interest.do_rectify = 4

    camin.height = 5
    camin.width = 4

    camin.distortion_model = "plumb_bob"

    camin.distortion.extend([3, 4, 5])

    camin.intrinsic_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
    camin.rectification_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11])
    camin.projection_matrix.extend([3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])

    camin.binning_x = 6
    camin.binning_y = 7

    camin.maximum_viewing_distance = 5

    stub.TransferCameraIntrinsics(camin)

    return camin


if __name__ == "__main__":
    sent_intrins = add_camintrins()
    # print the uuids of the project and the camera instrinsics
    print(
        f"camera instrinsics were saved with the uuid {sent_intrins.header.uuid_msgs} \
        on the project with the uuid {sent_intrins.header.uuid_project}"
    )

    # do the same for the distortion array
    print(f"camera instrinsics distortion array: {sent_intrins.distortion}")
