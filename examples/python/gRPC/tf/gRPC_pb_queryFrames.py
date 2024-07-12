#!/usr/bin/env python3
import sys
from typing import List

from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import frame_query_pb2 as frameQuery
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import string_vector_pb2
from seerep.pb import tf_service_pb2_grpc as tfService
from seerep.util.common import get_gRPC_channel


def get_frames(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[string_vector_pb2.StringVector]:
    stub = tfService.TfServiceStub(grpc_channel)
    stubMeta = metaOperations.MetaOperationsStub(grpc_channel)
    response = stubMeta.GetProjects(empty_pb2.Empty())

    if target_proj_uuid is None:
        # Get all projects from the server
        response = stubMeta.GetProjects(empty_pb2.Empty())
        for project in response.projects:
            # check for projects with the name "testproject"
            if project.name == "testproject":
                target_proj_uuid = project.uuid

        if target_proj_uuid is None:
            sys.exit()

    theQuery = frameQuery.FrameQuery()
    theQuery.projectuuid = target_proj_uuid
    frames: [List[string_vector_pb2.StringVector]] = stub.GetFrames(theQuery)
    return frames


if __name__ == "__main__":
    frames = get_frames()
    for frame in frames.stringVector:
        print("frame: " + frame)
