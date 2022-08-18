#!/usr/bin/env python3

import os
import sys

import meta_operations_pb2_grpc as metaOperations
import projectCreation_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = metaOperations.MetaOperationsStub(channel)
response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))

print("The new project on the server is (name/uuid):")
print("\t" + response.name + " " + response.uuid)
