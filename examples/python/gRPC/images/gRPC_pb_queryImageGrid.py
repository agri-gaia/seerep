#!/usr/bin/env python3

import os
import sys

import image_service_pb2_grpc
import labels_with_category_pb2
import meta_operations_pb2_grpc
import query_pb2 as query
from google.protobuf import empty_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = image_service_pb2_grpc.ImageServiceStub(channel)
stubMeta = meta_operations_pb2_grpc.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "LabeledImagesInGrid":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = query.Query()
theQuery.project_uuids.append(projectuuid)
theQuery.bounding_box_stamped.header.frame_id = "map"

theQuery.bounding_box_stamped.bounding_box.point_min.z = -1.0
theQuery.bounding_box_stamped.bounding_box.point_max.z = 1.0

# since epoche
theQuery.time_interval.time_min.seconds = 1638549273
theQuery.time_interval.time_min.nanos = 0
theQuery.time_interval.time_max.seconds = 1938549273
theQuery.time_interval.time_max.nanos = 0

# labels
label = labels_with_category_pb2.LabelsWithCategory()
label.category = "0"
label.labels.extend(["testlabel1"])
theQuery.categorized_labels.append(label)

for x in range(3):
    for y in range(3):
        theQuery.bounding_box_stamped.bounding_box.point_min.x = x - 0.5
        theQuery.bounding_box_stamped.bounding_box.point_min.y = y - 0.5
        theQuery.bounding_box_stamped.bounding_box.point_max.x = x + 0.5
        theQuery.bounding_box_stamped.bounding_box.point_max.y = y + 0.5
        for img in stub.GetImage(theQuery):
            print("General label of transferred img: " + img.labels_general[0].labelWithInstance[0].label)
