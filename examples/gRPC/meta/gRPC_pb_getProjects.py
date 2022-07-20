#!/usr/bin/env python3

import sys

import grpc
import meta_operations_pb2_grpc as metaOperations
from google.protobuf import empty_pb2

# # server with certs
# __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))) + "/../"
# with open(os.path.join(__location__, 'tls.pem'), 'rb') as f:
#     root_cert = f.read()
# server = "seerep.robot.10.249.3.13.nip.io:32141"
# creds = grpc.ssl_channel_credentials(root_cert)
# channel = grpc.secure_channel(server, creds)

# server without certs
server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = metaOperations.MetaOperationsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())


print(server + " has the following projects (name/uuid):")
for projectinfo in response.projects:
    print("\t" + projectinfo.name + " " + projectinfo.uuid)
