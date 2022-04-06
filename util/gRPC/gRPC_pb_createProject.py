#!/usr/bin/env python3

import sys

import grpc
import meta_operations_pb2_grpc as metaOperations
import projectCreation_pb2


with open('tls.pem', 'rb' ) as f:
    root_cert = f.read()

# server = "localhost:9090"
server = "seerep.robot.10.249.3.13.nip.io:32141"

# channel = grpc.insecure_channel(server)

creds = grpc.ssl_channel_credentials(root_cert)

channel = grpc.secure_channel(server, creds)

stub = metaOperations.MetaOperationsStub(channel)

response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))

print("The new project on the server " + server + " is (name/uuid):")
print("\t" + response.name + " " + response.uuid)
