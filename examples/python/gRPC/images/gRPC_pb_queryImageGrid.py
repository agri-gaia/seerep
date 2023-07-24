#!/usr/bin/env python3

import sys

from google.protobuf import empty_pb2
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import label_pb2
from seerep.pb import labels_with_category_pb2 as labels_with_category
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import query_pb2 as query
from seerep.util.common import get_gRPC_channel

channel = get_gRPC_channel()

stub = imageService.ImageServiceStub(channel)
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
theQuery.projectuuid.append(projectuuid)

bottom_left = point2d.Point2D()
bottom_left.x = -150
bottom_left.y = -150
theQuery.polygon.vertices.append(bottom_left)

top_left = point2d.Point2D()
top_left.x = -150
top_left.y = 150
theQuery.polygon.vertices.append(top_left)

top_right = point2d.Point2D()
top_right.x = 150
top_right.y = 150
theQuery.polygon.vertices.append(top_right)

bottom_right = point2d.Point2D()
bottom_right.x = 150
bottom_right.y = -150
theQuery.polygon.vertices.append(bottom_right)

theQuery.polygon.z = -1
theQuery.polygon.height = 7

# since epoche
theQuery.timeinterval.time_min.seconds = 1638549273
theQuery.timeinterval.time_min.nanos = 0
theQuery.timeinterval.time_max.seconds = 1938549273
theQuery.timeinterval.time_max.nanos = 0

# labels
label = labels_with_category.LabelsWithCategory()
label.category = "0"
labelWithConfidence = label_pb2.Label()
labelWithConfidence.label = "testlabel1"
label.labels.extend([labelWithConfidence])
theQuery.labelsWithCategory.append(label)

for x in range(3):
    for y in range(3):
        theQuery.boundingboxStamped.boundingbox.center_point.x = x
        theQuery.boundingboxStamped.boundingbox.center_point.y = y
        print("center point query (x/y): ( " + str(x) + " / " + str(y) + " )")
        for img in stub.GetImage(theQuery):
            print("General label of transferred img: " + img.labels_general[0].labelWithInstance[1].label.label)
            print("General label confidence: " + str(img.labels_general[0].labelWithInstance[1].label.confidence))
