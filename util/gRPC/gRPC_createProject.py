#!/usr/bin/env python3

import sys

# sys.path.insert(1, "/home/docker/workspace/build/seerep-msgs")
# sys.path.insert(1, "/home/docker/workspace/build/seerep-com")

import transfer_sensor_msgs_pb2_grpc as transferMsgs
import grpc
import projectCreation_pb2
import projectCreated_pb2

channel = grpc.insecure_channel("agrigaia-ur.ni.dfki:9090")

stub = transferMsgs.TransferSensorMsgsStub(channel)

response = stub.CreateProject(projectCreation_pb2.ProjectCreation(name="testproject", mapFrameId="map"))


print(response.name + " " + response.uuid)
