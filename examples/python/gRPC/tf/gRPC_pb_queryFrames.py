#!/usr/bin/env python3

import os
import sys

import frame_query_pb2
import meta_operations_pb2_grpc
import tf_service_pb2_grpc
from google.protobuf import empty_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = tf_service_pb2_grpc.TfServiceStub(channel)
stubMeta = meta_operations_pb2_grpc.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = frame_query_pb2.FrameQuery()
theQuery.project_uuid = projectuuid


frames = stub.GetFrames(theQuery)

for frame in frames.frames:
    print("frame: " + frame)
