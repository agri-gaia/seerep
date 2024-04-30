#!/usr/bin/env python3

from typing import List, Tuple

from google.protobuf import empty_pb2
from grpc import Channel
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.util.common import get_gRPC_channel


def get_projects(grpc_channel: Channel = get_gRPC_channel()) -> List[Tuple[str, str]]:
    stub = metaOperations.MetaOperationsStub(grpc_channel)

    response = stub.GetProjects(empty_pb2.Empty())

    projects_list: List[Tuple[str, str]] = []
    print("The server has the following projects (name/uuid):")

    for projectinfo in response.projects:
        projects_list.append((projectinfo.name, projectinfo.uuid))

    return projects_list


if __name__ == "__main__":
    project_list = get_projects()
    for proj in project_list:
        print("\t" + proj[0] + " " + proj[1])
