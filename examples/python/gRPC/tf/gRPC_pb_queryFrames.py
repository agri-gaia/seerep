#!/usr/bin/env python3
import sys

from google.protobuf import empty_pb2
from seerep.pb import frame_query_pb2 as frameQuery
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import tf_service_pb2_grpc as tfService
from seerep.util.common import get_gRPC_channel

channel = get_gRPC_channel()

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
