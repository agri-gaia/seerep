#!/usr/bin/env python3

import sys

# import numpy as np

import grpc
import transfer_sensor_msgs_pb2_grpc as transferMsgs
import image_pb2 as image
import query_pb2 as query

from google.protobuf import empty_pb2

channel = grpc.insecure_channel("agrigaia-ur.ni.dfki:9090")

stub = transferMsgs.TransferSensorMsgsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())
if not response.uuids:
    sys.exit()
else:
    print(response.uuids)
    projectname = response.uuids[0]


theQuery = query.Query()
theQuery.boundingbox.point_min.x = 0.0
theQuery.boundingbox.point_min.y = 0.0
theQuery.boundingbox.point_min.z = 0.0
theQuery.boundingbox.point_max.x = 10.0
theQuery.boundingbox.point_max.y = 10.0
theQuery.boundingbox.point_max.z = 10.0

# since epoche
theQuery.timeinterval.time_min = 1638549273
theQuery.timeinterval.time_max = 1938549273

# labels
theQuery.label.extend(["testlabel0"])


for img in stub.GetImage(theQuery):
    print("uuid of transfered img: " + img.labels_general[0])
