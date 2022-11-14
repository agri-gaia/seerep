#!/usr/bin/env python3

import os
import sys

import image_service_pb2_grpc as imageService
import meta_operations_pb2_grpc as metaOperations
import query_pb2 as query
from google.protobuf import empty_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid + "\n")
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = query.Query()
theQuery.projectuuid.append(projectuuid)
theQuery.boundingboxstamped.header.frame_id = "map"

theQuery.boundingboxstamped.boundingbox.point_min.x = 0.0
theQuery.boundingboxstamped.boundingbox.point_min.y = 0.0
theQuery.boundingboxstamped.boundingbox.point_min.z = 0.0
theQuery.boundingboxstamped.boundingbox.point_max.x = 100.0
theQuery.boundingboxstamped.boundingbox.point_max.y = 100.0
theQuery.boundingboxstamped.boundingbox.point_max.z = 100.0

# since epoche
theQuery.timeinterval.time_min.seconds = 1638549273
theQuery.timeinterval.time_min.nanos = 0
theQuery.timeinterval.time_max.seconds = 1938549273
theQuery.timeinterval.time_max.nanos = 0

# labels
theQuery.label.extend(["testlabel0"])


for img in stub.GetImage(theQuery):
    # currently not implemented #103
    # print(f"uuidmsg: {img.header.uuid_msgs}")
    print(f"first label: {img.labels_bb[0].labelWithInstance.label}")
    print(
        "First bounding box (Xmin, Ymin, Xmax, Ymax): "
        + str(img.labels_bb[0].boundingBox.point_min.x)
        + " "
        + str(img.labels_bb[0].boundingBox.point_min.y)
        + " "
        + str(img.labels_bb[0].boundingBox.point_max.x)
        + " "
        + str(img.labels_bb[0].boundingBox.point_max.y)
        + "\n"
    )
