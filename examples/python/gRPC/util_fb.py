import sys

import flatbuffers
from fb import Empty, ProjectCreation, ProjectInfo, ProjectInfos
from fb import meta_operations_grpc_fb as metaOperations


def get_or_create_project(channel, name, create=True, mapFrameId="map"):
    stubMeta = metaOperations.MetaOperationsStub(channel)

    builder = flatbuffers.Builder(1024)
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stubMeta.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    projectuuid = ""
    for i in range(response.ProjectsLength()):
        print(response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8") + "\n")
        if response.Projects(i).Name().decode("utf-8") == name:
            projectuuid = response.Projects(i).Uuid().decode("utf-8")

    if projectuuid == "":
        if create:
            mapFrameIdBuf = builder.CreateString(mapFrameId)
            nameBuf = builder.CreateString(name)
            ProjectCreation.Start(builder)
            ProjectCreation.AddMapFrameId(builder, mapFrameIdBuf)
            ProjectCreation.AddName(builder, nameBuf)
            projectCreationMsg = ProjectCreation.End(builder)
            builder.Finish(projectCreationMsg)
            buf = builder.Output()

            responseBuf = stubMeta.CreateProject(bytes(buf))
            response = ProjectInfo.ProjectInfo.GetRootAs(responseBuf)

            projectuuid = response.Uuid().decode("utf-8")
        else:
            sys.exit()

    return projectuuid
