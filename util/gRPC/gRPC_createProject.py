#!/usr/bin/env python3

import sys

import meta_operations_pb2_grpc as metaOperations
import grpc
import projectCreation_pb2

server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = metaOperations.MetaOperationsStub(channel)

response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))

print("The new project on the server " + server + " is (name/uuid):")
print("\t" + response.name + " " + response.uuid)
