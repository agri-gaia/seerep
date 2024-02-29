#!/usr/bin/env python3

from typing import Tuple

from grpc import Channel
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2
from seerep.util.common import get_gRPC_channel


def create_project(grpc_channel: Channel = get_gRPC_channel()) -> Tuple[str, str]:
    stub = metaOperations.MetaOperationsStub(grpc_channel)
    response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))

    return response.name, response.uuid


if __name__ == "__main__":
    ret = create_project()
    print("The new project on the server is (name/uuid):")
    print("\t" + ret[0] + " " + ret[1])
