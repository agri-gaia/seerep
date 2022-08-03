#!/usr/bin/env python3

import os
import sys

import grpc
import image_service_pb2_grpc as imageService
import meta_operations_pb2_grpc as metaOperations
import query_pb2 as query
from google.protobuf import empty_pb2

# server with certs
# __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
# with open(os.path.join(__location__, '../tls.pem'), 'rb') as f:
#     root_cert = f.read()
# server = "seerep.robot.10.249.3.13.nip.io:32141"
# creds = grpc.ssl_channel_credentials(root_cert)
# channel = grpc.secure_channel(server, creds)

# server without certs
channel = grpc.insecure_channel("localhost:9090")

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
theQuery.boundingbox.header.frame_id = "map"

theQuery.boundingbox.point_min.x = 0.0
theQuery.boundingbox.point_min.y = 0.0
theQuery.boundingbox.point_min.z = 0.0
theQuery.boundingbox.point_max.x = 100.0
theQuery.boundingbox.point_max.y = 100.0
theQuery.boundingbox.point_max.z = 100.0

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
    print(f"first abel: {img.labels_bb[0].labelWithInstance.label}")
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
