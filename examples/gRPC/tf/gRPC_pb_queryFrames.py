#!/usr/bin/env python3

import os
import sys

import frame_query_pb2 as frameQuery
import grpc
import meta_operations_pb2_grpc as metaOperations
import tf_service_pb2_grpc as tfService
from google.protobuf import empty_pb2

# import numpy as np

# # server with certs
# __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
# with open(os.path.join(__location__, '../tls.pem'), 'rb') as f:
#     root_cert = f.read()
# server = "seerep.robot.10.249.3.13.nip.io:32141"
# creds = grpc.ssl_channel_credentials(root_cert)
# channel = grpc.secure_channel(server, creds)

# server without certs
channel = grpc.insecure_channel("localhost:9090")

stub = tfService.TfServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = frameQuery.FrameQuery()
theQuery.projectuuid = projectuuid


frames = stub.GetFrames(theQuery)

for frame in frames.frames:
    print("frame: " + frame)
