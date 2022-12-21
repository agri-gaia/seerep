#!/usr/bin/env python3

import os
import sys

import meta_operations_pb2_grpc
import project_creation_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = meta_operations_pb2_grpc.MetaOperationsStub(channel)
response = stub.CreateProject(project_creation_pb2.ProjectCreation(name="testproject", map_frame_id="map"))

print("The new project on the server is (name/uuid):")
print("\t" + response.name + " " + response.uuid)
