#!/usr/bin/env python3

from google.protobuf import empty_pb2
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.util.common import get_gRPC_channel

channel = get_gRPC_channel()

stub = metaOperations.MetaOperationsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())


print("The server has the following projects (name/uuid):")
for projectinfo in response.projects:
    print("\t" + projectinfo.name + " " + projectinfo.uuid)
