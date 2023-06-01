#!/usr/bin/env python3

from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import projectCreation_pb2
from seerep.util.common import get_gRPC_channel

channel = get_gRPC_channel()

stub = metaOperations.MetaOperationsStub(channel)
response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))

print("The new project on the server is (name/uuid):")
print("\t" + response.name + " " + response.uuid)
