#!/usr/bin/env python3
from typing import Optional

from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import camera_intrinsics_query_pb2 as cameraintrinsicsquery
from seerep.pb import (
    camera_intrinsics_service_pb2_grpc as camintrinsics_service,
)
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.util.common import get_gRPC_channel


def query_camintrins(
    ciuuid: str = "fa2f27e3-7484-48b0-9f21-ec362075baca",
    target_proj_uuid: Optional[str] = None,
    grpc_channel: Channel = get_gRPC_channel(),
):
    if target_proj_uuid is None:
        # 2. Get all projects from the server
        stubMeta = metaOperations.MetaOperationsStub(grpc_channel)
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            print(project.name + " " + project.uuid)
            if project.name == "testproject":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            print("""
                valid project doesn't exist! Please execute
                gRPC_pb_addCameraIntrinsics.py beforehand.
            """)

    stub = camintrinsics_service.CameraIntrinsicsServiceStub(grpc_channel)

    # Fetch the CI
    ci_query = cameraintrinsicsquery.CameraIntrinsicsQuery()

    ci_query.uuid_camera_intrinsics = ciuuid
    ci_query.uuid_project = target_proj_uuid

    fetched_camintrinsics = stub.GetCameraIntrinsics(ci_query)
    print(fetched_camintrinsics)
    return fetched_camintrinsics


if __name__ == "__main__":
    queried_camintrinsics = query_camintrins()
    # for verification print the uuid of the added camera intrinsics
    print(
        f"camera instrinsics were saved with the uuid \
        {queried_camintrinsics.header.uuid_msgs}"
        f" on the project with the uuid \
        {queried_camintrinsics.header.uuid_project}"
    )
    # print the distortion of the retrieved camera intrinsics
    print(
        f"camera intrinsics distortion array: \
            {queried_camintrinsics.distortion}"
    )
