#!/usr/bin/env python3

import flatbuffers
import grpc
from seerep.fb import Empty, ProjectInfos
from seerep.fb import metaOperations_grpc_fb as metaOperations

server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)
Empty.Start(builder)
emptyMsg = Empty.End(builder)
builder.Finish(emptyMsg)
buf = builder.Output()

responseBuf = stub.GetProjects(bytes(buf))
response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

print(server + " has the following projects (name/uuid):")
for i in range(response.ProjectsLength()):
    print("\t" + response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8"))
