#!/usr/bin/env python3

from typing import List, Tuple

from google.protobuf import empty_pb2
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.util.common import get_gRPC_channel


def get_projects() -> List[Tuple[str, str]]:
    channel = get_gRPC_channel()

    stub = metaOperations.MetaOperationsStub(channel)

    response = stub.GetProjects(empty_pb2.Empty())

    projects_list: List[Tuple[str, str]] = []
    print("The server has the following projects (name/uuid):")

    for projectinfo in response.projects:
        print("\t" + projectinfo.name + " " + projectinfo.uuid)
        projects_list.append((projectinfo.name, projectinfo.uuid))

    return projects_list


if __name__ == "__main__":
    get_projects()
