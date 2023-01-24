#!/usr/bin/env python3

import os
import sys

import meta_operations_pb2_grpc as metaOperations
from google.protobuf import empty_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = metaOperations.MetaOperationsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())

print("The server has the following projects (name/uuid):")
for projectinfo in response.projects:
    print("\t" + projectinfo.name + " " + projectinfo.uuid)
