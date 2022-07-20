#!/usr/bin/env python3

import os

import flatbuffers
import grpc
from seerep.fb import Empty, ProjectInfos
from seerep.fb import meta_operations_grpc_fb as metaOperations

# # server with certs
# __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
# with open(os.path.join(__location__, 'tls.pem'), 'rb') as f:
#     root_cert = f.read()
# server = "seerep.robot.10.249.3.13.nip.io:32141"
# creds = grpc.ssl_channel_credentials(root_cert)
# channel = grpc.secure_channel(server, creds)

# server without certs
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
