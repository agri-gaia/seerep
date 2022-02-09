#!/usr/bin/env python3

import sys

import meta_operations_pb2_grpc as metaOperations
import grpc
import projectCreation_pb2

channel = grpc.insecure_channel("agrigaia-ur.ni.dfki:9090")

stub = metaOperations.MetaOperationsStub(channel)

response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))


print(response.name + " " + response.uuid)
