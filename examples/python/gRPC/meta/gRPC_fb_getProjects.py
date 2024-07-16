#!/usr/bin/env python3
# NOTE: This file is referenced in the following mkdocs files:
#   projects.md
# Any changes done in here will be reflected in there
from typing import List

import flatbuffers
from grpc import Channel
from seerep.fb import Empty, ProjectInfo, ProjectInfos
from seerep.fb import meta_operations_grpc_fb as metaOperations
from seerep.util.common import get_gRPC_channel


def get_projects_raw(
    grpc_channel: Channel = get_gRPC_channel(),
) -> bytearray:
    """
    Returns: bytearray of type ProjectInfos
    """
    stub = metaOperations.MetaOperationsStub(grpc_channel)

    builder = flatbuffers.Builder(1024)
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stub.GetProjects(bytes(buf))
    return responseBuf


def get_projects(
    grpc_channel: Channel = get_gRPC_channel(),
) -> ProjectInfos.ProjectInfos:
    return ProjectInfos.ProjectInfos.GetRootAs(get_projects_raw(grpc_channel))


if __name__ == "__main__":
    response = get_projects()

    projects_list: List[ProjectInfo.ProjectInfo] = []

    for i in range(response.ProjectsLength()):
        projects_list.append(response.Projects(i))

    print("The server has the following projects (name/uuid):")
    for project in projects_list:
        print(
            "\t"
            + project.Name().decode("utf-8")
            + " "
            + project.Uuid().decode("utf-8")
        )
