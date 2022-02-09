#!/usr/bin/env python3

import sys

# import numpy as np

import grpc
import query_data_pb2_grpc as queryData
import meta_operations_pb2_grpc as metaOperations
import image_pb2 as image
import query_pb2 as query

from google.protobuf import empty_pb2

channel = grpc.insecure_channel("agrigaia-ur.ni.dfki:9090")

stub = queryData.QueryDataStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "LabeledImagesInGrid":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = query.Query()
theQuery.projectuuid = projectuuid
theQuery.boundingbox.header.frame_id = "map"

theQuery.boundingbox.point_min.z = -1.0
theQuery.boundingbox.point_max.z = 1.0

# since epoche
theQuery.timeinterval.time_min = 1638549273
theQuery.timeinterval.time_max = 1938549273

# labels
theQuery.label.extend(["http://aims.fao.org/aos/agrovoc/c_24596"])

for x in range(3):
    for y in range(3):
        theQuery.boundingbox.point_min.x = x - 0.5
        theQuery.boundingbox.point_min.y = y - 0.5
        theQuery.boundingbox.point_max.x = x + 0.5
        theQuery.boundingbox.point_max.y = y + 0.5
        for img in stub.GetImage(theQuery):
            print("label general 0 of transfered img: " + img.labels_general[0])
