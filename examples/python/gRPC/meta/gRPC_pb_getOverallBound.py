#!/usr/bin/env python3

import os
import sys

import seerep.pb.datatype_pb2 as datatype
import seerep.pb.meta_operations_pb2_grpc as metaOperations
import seerep.pb.uuid_datatype_pair_pb2 as uuid_datatype_pair
from google.protobuf import empty_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = metaOperations.MetaOperationsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())

# 3. Check if we have an existing test project, if not, we stop here
projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid + "\n")
    if project.name == "testproject":
        projectuuid = project.uuid

uuiddt = uuid_datatype_pair.UuidDatatypePair()
uuiddt.projectuuid = projectuuid
uuiddt.datatype = datatype.image

response = stub.GetOverallTimeInterval(uuiddt)
print(response)

response = stub.GetOverallBoundingBox(uuiddt)
print(response)