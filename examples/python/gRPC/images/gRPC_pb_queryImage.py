#!/usr/bin/env python3

import os
import sys

import image_service_pb2_grpc
import labels_with_category_pb2
import meta_operations_pb2_grpc
import query_pb2
from google.protobuf import empty_pb2

# importing util functions. Assuming that this file is in the parent dir
# https://github.com/agri-gaia/seerep/blob/6c4da5736d4a893228e97b01a9ada18620b1a83f/examples/python/gRPC/util.py
script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

# Default server is localhost !
channel = util.get_gRPC_channel()

# 1. Get gRPC service objects
stub = image_service_pb2_grpc.ImageServiceStub(channel)
stubMeta = meta_operations_pb2_grpc.MetaOperationsStub(channel)

# 2. Get all projects from the server
response = stubMeta.GetProjects(empty_pb2.Empty())

# 3. Check if we have an existing test project, if not, we stop here
projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid + "\n")
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()

# 4. Create a query with parameters
theQuery = query_pb2.Query()
theQuery.project_uuids.append(projectuuid)
theQuery.bounding_box_stamped.header.frame_id = "map"

theQuery.bounding_box_stamped.bounding_box.point_min.x = 0.0
theQuery.bounding_box_stamped.bounding_box.point_min.y = 0.0
theQuery.bounding_box_stamped.bounding_box.point_min.z = 0.0
theQuery.bounding_box_stamped.bounding_box.point_max.x = 100.0
theQuery.bounding_box_stamped.bounding_box.point_max.y = 100.0
theQuery.bounding_box_stamped.bounding_box.point_max.z = 100.0

# since epoche
theQuery.time_interval.time_min.seconds = 1638549273
theQuery.time_interval.time_min.nanos = 0
theQuery.time_interval.time_max.seconds = 1938549273
theQuery.time_interval.time_max.nanos = 0

# labels
label = labels_with_category_pb2.LabelsWithCategory()
label.category = "0"
label.labels.extend(["testlabel0"])
theQuery.categorized_labels.append(label)

# 5. Query the server for images matching the query and iterate over them
for img in stub.GetImage(theQuery):
    print(f"uuidmsg: {img.header.uuid_msgs}")
    print(f"first label: {img.labeled_bounding_boxes[0].labeled_2d_bounding_boxes[0].label_with_instance.label}")
    print(
        "First bounding box (Xmin, Ymin, Xmax, Ymax): "
        + str(img.labeled_bounding_boxes[0].labeled_2d_bounding_boxes[0].bounding_box.point_min.x)
        + " "
        + str(img.labeled_bounding_boxes[0].labeled_2d_bounding_boxes[0].bounding_box.point_min.y)
        + " "
        + str(img.labeled_bounding_boxes[0].labeled_2d_bounding_boxes[0].bounding_box.point_max.x)
        + " "
        + str(img.labeled_bounding_boxes[0].labeled_2d_bounding_boxes[0].bounding_box.point_max.y)
        + "\n"
    )
