#!/usr/bin/env python3

import flatbuffers
from seerep.fb import Empty, ProjectInfos
from seerep.fb import meta_operations_grpc_fb as metaOperations
from seerep.util.common import get_gRPC_channel

channel = get_gRPC_channel()

stub = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)
Empty.Start(builder)
emptyMsg = Empty.End(builder)
builder.Finish(emptyMsg)
buf = builder.Output()

responseBuf = stub.GetProjects(bytes(buf))
response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

print("The server has the following projects (name/uuid):")
for i in range(response.ProjectsLength()):
    print("\t" + response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8"))
