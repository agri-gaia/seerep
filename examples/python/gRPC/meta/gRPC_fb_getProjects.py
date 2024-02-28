#!/usr/bin/env python3
from typing import List

import flatbuffers
from grpc import Channel
from seerep.fb import Empty, ProjectInfo, ProjectInfos
from seerep.fb import meta_operations_grpc_fb as metaOperations
from seerep.util.common import get_gRPC_channel


def get_projects(
    grpc_channel: Channel = get_gRPC_channel(),
) -> List[ProjectInfo.ProjectInfo]:
    stub = metaOperations.MetaOperationsStub(grpc_channel)

    builder = flatbuffers.Builder(1024)
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stub.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    projects_list: List[ProjectInfo.ProjectInfo] = []

    for i in range(response.ProjectsLength()):
        projects_list.append(response.Projects(i))

    return projects_list


if __name__ == "__main__":
    projects = get_projects()

    print("The server has the following projects (name/uuid):")
    for project in projects:
        print("\t" + project.Name().decode("utf-8") + " " + project.Uuid().decode("utf-8"))
